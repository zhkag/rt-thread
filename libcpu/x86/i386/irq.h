/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-16     JasonHu      first version
 */

#ifndef __IRQ_H__
#define __IRQ_H__

#define IRQ0_CLOCK          0
#define IRQ1_KEYBOARD       1
#define IRQ2_CONNECT        2   /* connect to slave */
#define IRQ3_SERIAL2        3
#define IRQ4_SERIAL1        4
#define IRQ5_PARALLEL2      5
#define IRQ6_FLOPPY         6
#define IRQ7_PARALLEL1      7

#define IRQ8_RTCLOCK        8   /* real-time clock */
#define IRQ9_REDIRECT       9   /* redirect to IRQ2 */
#define IRQ10_RESERVED      10
#define IRQ11_RESERVED      11
#define IRQ12_MOUSE         12
#define IRQ13_FPU           13
#define IRQ14_HARDDISK      14
#define IRQ15_RESERVE       15

#endif /* __IRQ_H__ */
