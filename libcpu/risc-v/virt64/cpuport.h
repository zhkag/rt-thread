/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-03     Bernard      The first version
 */

#ifndef CPUPORT_H__
#define CPUPORT_H__

#include <rtconfig.h>

/* bytes of register width  */
#ifdef ARCH_CPU_64BIT
#define STORE                   sd
#define LOAD                    ld
#define REGBYTES                8
#else
// error here, not portable
#endif

/* 33 normal register */
#define CTX_REG_NR  33

#ifndef __ASSEMBLY__
rt_inline void rt_hw_dsb()
{
    asm volatile("fence":::"memory");
}

rt_inline void rt_hw_dmb()
{
    asm volatile("fence":::"memory");
}

rt_inline void rt_hw_isb()
{
    asm volatile("fence.i":::"memory");
}

#endif

#endif
#ifdef RISCV_U_MODE
#define RISCV_USER_ENTRY 0xFFFFFFE000000000ULL
#endif