/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-19     JasonHu      first version
 */

#ifndef __RISCV64_PLIC_H__
#define __RISCV64_PLIC_H__

// Platform level interrupt controller
#define PLIC                    0x10000000L

#define PLIC_PRIORITY           (PLIC + 0x0)
#define PLIC_PENDING            (PLIC + 0x1000)
#define PLIC_MENABLE(hart)      (PLIC + 0x2000 + (hart) * 0x100)      // machine interrupt enable
#define PLIC_SENABLE(hart)      (PLIC + 0x2080 + (hart) * 0x100)      // supervisor interrupt enable
#define PLIC_MPRIORITY(hart)    (PLIC + 0x200000 + (hart) * 0x2000)
#define PLIC_SPRIORITY(hart)    (PLIC + 0x201000 + (hart) * 0x2000)
#define PLIC_MCLAIM(hart)       (PLIC + 0x200004 + (hart) * 0x2000)
#define PLIC_SCLAIM(hart)       (PLIC + 0x201004 + (hart) * 0x2000)

void plic_init(void);
void plic_enable_irq(int irqno);
void plic_disable_irq(int irqno);
// ask PLIC what interrupt we should serve
int plic_claim(void);
// tell PLIC that we've served this IRQ
void plic_complete(int irq);

#endif
