/**
 * @file    lib_typedef.h
 * @brief   通用类型定义（不含 MCU 头文件）
 */
#ifndef LIB_TYPEDEF_H
#define LIB_TYPEDEF_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* ─── 字节打包工具 (统一入口, 避免各模块重复定义) ─── */

#define LIB_HI_BYTE(x)  ((uint8_t)((uint16_t)(x) >> 8))
#define LIB_LO_BYTE(x)  ((uint8_t)(x))

#endif