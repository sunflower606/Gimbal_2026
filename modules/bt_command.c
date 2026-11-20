#include "bt_command.h"

#include "bsp_bt_uart.h"

/* 一行最长多少字符（"PITCH 177.5" 也只有 11 个，留足余量） */
#define BT_CMD_LINE_MAX     20U

static char s_line[BT_CMD_LINE_MAX];
static uint8_t s_len = 0U;
static uint8_t s_overflow = 0U;     /* 1 = 本行太长或含不可打印字符，整行作废 */

/* 最近一次解析出来的数字参数（只有 yaw/pitch 会写它） */
static float s_parse_arg = 0.0f;

static char BtCommand_Upper(char c)
{
    if ((c >= 'a') && (c <= 'z')) {
        return (char)(c - 'a' + 'A');
    }
    return c;
}

static uint8_t BtCommand_Equals(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0')) {
        if (*a != *b) {
            return 0U;
        }
        a++;
        b++;
    }
    return ((*a == '\0') && (*b == '\0')) ? 1U : 0U;
}

/* a 是否以 prefix 开头 */
static uint8_t BtCommand_StartsWith(const char *a, const char *prefix)
{
    while (*prefix != '\0') {
        if (*a != *prefix) {
            return 0U;
        }
        a++;
        prefix++;
    }
    return 1U;
}

/**
 * @brief 把一个十进制字符串（可带负号、可带一位以上小数）转成浮点。
 * @return 1 = 成功；0 = 空串或含非法字符。
 *
 * @note 不用 strtof：newlib-nano 里它会把一大批浮点转换代码链进来
 *       （本工程 FLASH 只有 64 KB，目前已用 28%）。
 */
static uint8_t BtCommand_ParseNumber(const char *s, float *out)
{
    uint32_t int_part = 0U;
    uint32_t frac_part = 0U;
    uint32_t scale = 1U;
    uint8_t negative = 0U;
    uint8_t digits = 0U;
    float value;

    /* 数字前的分隔符一律跳掉：允许 "yaw 100" / "yaw:100" / "yaw=100" */
    while ((*s == ':') || (*s == '=') || (*s == ' ') || (*s == '\t')) {
        s++;
    }

    if (*s == '-') {
        negative = 1U;
        s++;
    } else if (*s == '+') {
        s++;
    }

    while ((*s >= '0') && (*s <= '9')) {
        int_part = (int_part * 10U) + (uint32_t)(*s - '0');
        digits = 1U;
        s++;
        if (int_part > 1000U) {
            return 0U;      /* 明显超出舵机行程，直接当非法输入 */
        }
    }

    if (*s == '.') {
        s++;
        while ((*s >= '0') && (*s <= '9') && (scale < 100U)) {
            frac_part = (frac_part * 10U) + (uint32_t)(*s - '0');
            scale *= 10U;
            digits = 1U;
            s++;
        }
    }

    if ((digits == 0U) || (*s != '\0')) {
        return 0U;
    }

    value = (float)int_part + ((float)frac_part / (float)scale);
    if (negative != 0U) {
        value = -value;
    }
    *out = value;
    return 1U;
}

/* 把行缓冲判成命令：先去掉空格/制表符、转大写，再逐个比对 */
static BtCommand_t BtCommand_Parse(void)
{
    char norm[BT_CMD_LINE_MAX];
    const char *num;
    uint8_t n = 0U;
    uint8_t i;

    s_parse_arg = 0.0f;

    for (i = 0U; i < s_len; i++) {
        char c = s_line[i];

        if ((c == ' ') || (c == '\t')) {
            continue;
        }
        if (n >= (BT_CMD_LINE_MAX - 1U)) {
            return BT_CMD_NONE;
        }
        norm[n] = BtCommand_Upper(c);
        n++;
    }
    norm[n] = '\0';

    if (n == 0U) {
        return BT_CMD_NONE;
    }

    if ((BtCommand_Equals(norm, "MODE1") != 0U) || (BtCommand_Equals(norm, "M1") != 0U)) {
        return BT_CMD_MODE1;
    }
    if ((BtCommand_Equals(norm, "MODE2") != 0U) || (BtCommand_Equals(norm, "M2") != 0U)) {
        return BT_CMD_MODE2;
    }
    if ((BtCommand_Equals(norm, "STOP") != 0U) || (BtCommand_Equals(norm, "S") != 0U)) {
        return BT_CMD_STOP;
    }

    /* yaw <角度> */
    if (BtCommand_StartsWith(norm, "YAW") != 0U) {
        num = &norm[3];
        if (BtCommand_ParseNumber(num, &s_parse_arg) != 0U) {
            return BT_CMD_YAW;
        }
        return BT_CMD_NONE;
    }

    /* pitch <角度>，也接受简写 pit */
    if (BtCommand_StartsWith(norm, "PIT") != 0U) {
        num = &norm[3];
        if ((num[0] == 'C') && (num[1] == 'H')) {
            num += 2;
        }
        if (BtCommand_ParseNumber(num, &s_parse_arg) != 0U) {
            return BT_CMD_PITCH;
        }
        return BT_CMD_NONE;
    }

    return BT_CMD_NONE;
}

void BtCommand_Init(void)
{
    s_len = 0U;
    s_overflow = 0U;
    s_parse_arg = 0.0f;
}

BtCommandEvent_t BtCommand_Poll(void)
{
    BtCommandEvent_t event;
    uint8_t byte;

    event.command = BT_CMD_NONE;
    event.argument = 0.0f;

    while (BSP_BT_ReadByte(&byte) != 0U) {
        if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n')) {
            if (s_overflow == 0U) {
                BtCommand_t parsed = BtCommand_Parse();

                if (parsed != BT_CMD_NONE) {
                    /* 一拍里收到多条，以最后一条为准 */
                    event.command = parsed;
                    event.argument = s_parse_arg;
                }
            }
            s_len = 0U;
            s_overflow = 0U;
            continue;
        }

        /* 只收可打印 ASCII；终端里常见的控制字符直接让本行作废 */
        if ((byte < 0x20U) || (byte > 0x7EU)) {
            s_overflow = 1U;
            continue;
        }
        if (s_len >= (BT_CMD_LINE_MAX - 1U)) {
            s_overflow = 1U;
            continue;
        }

        s_line[s_len] = (char)byte;
        s_len++;
    }

    return event;
}

const char *BtCommand_ToString(BtCommand_t command)
{
    switch (command) {
    case BT_CMD_MODE1:
        return "mode1";
    case BT_CMD_MODE2:
        return "mode2";
    case BT_CMD_STOP:
        return "stop";
    case BT_CMD_YAW:
        return "yaw";
    case BT_CMD_PITCH:
        return "pitch";
    default:
        return "none";
    }
}
