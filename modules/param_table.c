#include "param_table.h"

#include <stddef.h>

/* 短名 -> 编号。编号与 Chassis_2026/modules/params.h 一一对应。
 * 名字都存大写，查找时调用方负责先转大写。 */
static const ParamName_t s_table[] = {
    { "SG",   0U,  "permille/deg" },
    { "SDB",  1U,  "0.1deg" },
    { "SMAX", 2U,  "permille" },
    { "DG",   3U,  "permille/cm" },
    { "VMAX", 4U,  "permille" },
    { "BASE", 5U,  "permille" },
    { "DDB",  6U,  "cm" },
    { "KD",   7U,  "permille/(deg/s)" },
    { "TO",   8U,  "0.1deg" },
    { "LG",   9U,  "coef" },
    { "LV",  10U,  "permille" },
    { "BD",  11U,  "0.1deg" },
    { "BS",  12U,  "permille" },
    { "A2",  13U,  "0=off 1=on" },
    { "LS",  14U,  "percent" },
    { "LH",  15U,  "ms" },
    { "LKD", 16U,  "permille" },
    { "CRS", 17U,  "ms" }
};

#define PARAM_TABLE_COUNT   ((uint8_t)(sizeof(s_table) / sizeof(s_table[0])))

static uint8_t ParamTable_Equals(const char *a, const char *b)
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

uint16_t ParamTable_Find(const char *name)
{
    uint8_t i;

    for (i = 0U; i < PARAM_TABLE_COUNT; i++) {
        if (ParamTable_Equals(s_table[i].name, name) != 0U) {
            return s_table[i].id;
        }
    }
    return 0xFFFFU;
}

uint8_t ParamTable_Count(void)
{
    return PARAM_TABLE_COUNT;
}

const ParamName_t *ParamTable_At(uint8_t index)
{
    if (index >= PARAM_TABLE_COUNT) {
        return NULL;
    }
    return &s_table[index];
}
