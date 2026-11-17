#include "bt_command.h"

#include "bsp_bt_uart.h"

/* 一行最长多少字符（"mode2" 只有 5 个，留足余量给前后缀） */
#define BT_CMD_LINE_MAX     12U

static char s_line[BT_CMD_LINE_MAX];
static uint8_t s_len = 0U;
static uint8_t s_overflow = 0U;     /* 1 = 本行太长或含不可打印字符，整行作废 */

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

/* 把行缓冲判成命令码：先去掉空格/制表符、转大写，再逐个比对 */
static BtCommand_t BtCommand_Parse(void)
{
    char norm[BT_CMD_LINE_MAX];
    uint8_t n = 0U;
    uint8_t i;

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

    return BT_CMD_NONE;
}

void BtCommand_Init(void)
{
    s_len = 0U;
    s_overflow = 0U;
}

BtCommand_t BtCommand_Poll(void)
{
    BtCommand_t result = BT_CMD_NONE;
    uint8_t byte;

    while (BSP_BT_ReadByte(&byte) != 0U) {
        if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n')) {
            if (s_overflow == 0U) {
                BtCommand_t parsed = BtCommand_Parse();

                if (parsed != BT_CMD_NONE) {
                    result = parsed;    /* 一拍里收到多条，以最后一条为准 */
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

    return result;
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
    default:
        return "none";
    }
}
