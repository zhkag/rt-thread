/*
 * File      : board.h
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-07-06     Bernard    the first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <rtconfig.h>
#include <stdlib.h>
#include "mmu.h"
#include "ioremap.h"

extern unsigned char __bss_start;
extern unsigned char __bss_end;

#define HEAP_BEGIN      ((void*)&__bss_end)

#ifdef RT_USING_USERSPACE
#define HEAP_END        ((size_t)KERNEL_VADDR_START + 16 * 1024 * 1024)
#define PAGE_START      HEAP_END
#define PAGE_END        ((size_t)KERNEL_VADDR_START + 128 * 1024 * 1024)
#else
#define HEAP_END        ((size_t)0x40000000 + 64 * 1024 * 1024)
#endif

#define __REG32(x)  (*((volatile unsigned int *)(x)))
#define __REG16(x)  (*((volatile unsigned short *)(x)))

/* UART PL011 */
#define PL011_UARTDR                (0x000)
#define PL011_UARTFR                (0x018)
#define PL011_UARTFR_TXFF_BIT       (5)
#define PL011_UART0_BASE            (0x09000000)
#define PL011_UART0_SIZE            (0x00001000)
#define PL011_UART0_IRQNUM          (33)

/* GIC PL390 DIST and CPU */
#define GIC_PL390_DISTRIBUTOR_PPTR  (0x08000000)
#define GIC_PL390_CONTROLLER_PPTR   (0x08010000)

#define MAX_HANDLERS     (96)
#define GIC_IRQ_START   (0)
/* number of interrupts on board */
#define ARM_GIC_NR_IRQS     (96)
/* only one GIC available */
#define ARM_GIC_MAX_NR      (1)

#define TIMER_IRQ   (30)

/* the basic constants and interfaces needed by gic */
rt_inline rt_uint64_t platform_get_gic_dist_base(void)
{
    return GIC_PL390_DISTRIBUTOR_PPTR;
}

rt_inline rt_uint64_t platform_get_gic_cpu_base(void)
{
    return GIC_PL390_CONTROLLER_PPTR;
}

void rt_hw_board_init(void);

extern rt_mmu_info mmu_info;

#endif
