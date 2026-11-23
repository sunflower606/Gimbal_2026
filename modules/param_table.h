#ifndef PARAM_TABLE_H
#define PARAM_TABLE_H

/* ============================================================================
 * Module：参数名表（云台板侧）
 *
 * 作用只有一个：把上位机敲的短名（"sg"）翻译成参数编号（0），
 * 再交给链路帧 0x03 发给循迹板。真正的数值、范围检查都在循迹板，
 * 这里不做任何范围判断 —— 免得两边各有一套规则、对不上时难查。
 *
 * 表里的编号必须与 Chassis_2026/modules/params.h 以及
 * docs/双板分工与协议.md 完全一致。
 * ========================================================================== */

#include <stdint.h>

/** @brief 一条参数名到编号的映射。 */
typedef struct {
    const char *name;   /* 短名，大小写不敏感（这里存大写） */
    uint16_t id;        /* 参数编号，0~63 归循迹板 */
    const char *unit;   /* 单位，仅用于 dump 显示 */
} ParamName_t;

/** @return 名字对应的编号；找不到返回 0xFFFF。 */
uint16_t ParamTable_Find(const char *name);

/** @return 参数总数。 */
uint8_t ParamTable_Count(void);

/** @brief 按下标取一项，越界返回 NULL。 */
const ParamName_t *ParamTable_At(uint8_t index);

#endif /* PARAM_TABLE_H */
