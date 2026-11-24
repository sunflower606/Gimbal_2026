#ifndef BT_COMMAND_H
#define BT_COMMAND_H

/* ============================================================================
 * Module：手机经蓝牙发给云台板的文本命令
 *
 * 场景：手机用蓝牙串口发 "mode2"，让循迹板进入模式 2（纯人体跟随）；
 *       发 "yaw 200"，让云台直接转到 200 度（标零点、量视场、录演示用）。
 *
 * 为什么命令在云台板解析、而不是原样转给循迹板：
 *   蓝牙模块接在云台板的 USART3 上，而循迹板那条 USART1 上跑的是
 *   13 字节二进制帧。把 ASCII 文本混进同一条线，会出现「文本里恰好有
 *   AA 55 02 08」和「帧里恰好有 mode2」两种误判，调试时无法区分。
 *   所以在这里解析成命令码，占链路帧第 11 字节（原本保留为 0）下发，
 *   二进制帧里永远只有二进制。
 *
 * 支持的命令（大小写不敏感，行首尾空格/制表符忽略，CR 或 LF 结尾都行，
 * 数字前后的空格、冒号、等号也都被忽略）：
 *   mode1 / m1        -> 1  循迹模式（下发给循迹板）
 *   mode2 / m2        -> 2  跟随模式（下发给循迹板）
 *   stop  / s         -> 3  停车（下发给循迹板，锁存到位再发 mode）
 *   yaw 100 / yaw100  -> 4  云台 yaw 转到 100 度（本板执行，不下发）
 *   pitch 75          -> 5  云台 pitch 转到 75 度（本板执行，不下发）
 *   set <名字> <值>   -> 6  改循迹板参数，转发成 0x03 帧（例 set sg 6）
 *   get <名字>        -> 7  读一个参数（例 get sg）
 *   dump              -> 8  列出全部参数
 *   save              -> 9  存 Flash（A2，暂回"未支持"）
 *   reset             -> 10 恢复出厂默认值
 *   命令后面不带数字（只发 yaw）视为无效，不会动云台。
 * ========================================================================== */

#include <stdint.h>

/** @brief 命令码。1/2/3 会写进链路帧第 11 字节；4/5 只在本板执行。 */
typedef enum {
    BT_CMD_NONE = 0,    /* 没有新命令 */
    BT_CMD_MODE1 = 1,   /* 切模式 1：循迹 */
    BT_CMD_MODE2 = 2,   /* 切模式 2：跟随 */
    BT_CMD_STOP = 3,    /* 停车（锁存） */
    BT_CMD_YAW = 4,     /* 设置 yaw 角度，带参数 */
    BT_CMD_PITCH = 5,   /* 设置 pitch 角度，带参数 */
    BT_CMD_SET = 6,     /* set <名字> <值>：改循迹板上的参数 */
    BT_CMD_GET = 7,     /* get <名字>：读一个参数 */
    BT_CMD_DUMP = 8,    /* dump：列出全部参数 */
    BT_CMD_SAVE = 9,    /* save：参数存 Flash（A2 功能，暂未实现） */
    BT_CMD_RESET = 10,  /* reset：参数恢复默认 */
    BT_CMD_A2 = 11,     /* f（单个字符）：切换「参数存 Flash」开关 */
    BT_CMD_BEEPER = 12, /* bz on|off|（空）：蜂鸣器开关 / 查询 */
    BT_CMD_BB = 13      /* bb：导出循迹板里的黑匣子日志 */
} BtCommand_t;

/** @brief 一条命令 + 它的参数（只有 yaw/pitch 用得到参数）。 */
typedef struct {
    BtCommand_t command;    /* 命令码；BT_CMD_NONE 表示这一拍没新命令 */
    float argument;         /* 角度（yaw/pitch）或数值（set），其它命令恒为 0 */
    char name[8];           /* set/get 的参数短名（大写），其它命令为空串 */
} BtCommandEvent_t;

/* 清空行缓冲（上电调用一次） */
void BtCommand_Init(void);

/**
 * @brief 把蓝牙收到的字节全部取出来拼行，返回本次识别到的最后一条命令。
 * @return 命令 + 参数；command 为 BT_CMD_NONE 表示这一拍没有新命令。
 *
 * @note 非阻塞、不等待；由云台主循环每拍调用一次。
 */
BtCommandEvent_t BtCommand_Poll(void);

/** @brief 命令码转文本（给回执用）。 */
const char *BtCommand_ToString(BtCommand_t command);

#endif /* BT_COMMAND_H */
