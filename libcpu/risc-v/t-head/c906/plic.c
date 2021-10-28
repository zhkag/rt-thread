/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-19     JasonHu      first version
 */

#include <rtthread.h>
#include <cpuport.h>

#include <rtdbg.h>

#include "plic.h"
#include "rt_interrupt.h"
#include "io.h"

void plic_enable_irq(int irqno)
{
    if (irqno < 0 || irqno >= IRQ_MAX_NR) {
        LOG_E("[IRQ] plic enable bad irq %d!", irqno);
        return;
    }
    int hart = rt_hw_cpu_id();

    uint32_t *enable_addr = (uint32_t *)PLIC_SENABLE(hart);

    enable_addr += irqno / 32;
    uint8_t irqno_off = irqno % 32;
    // set uart's enable bit for this hart's S-mode.
    writel(readl((void *)enable_addr) | (1 << irqno_off), enable_addr);

    uint32_t *priority_addr = (uint32_t *) PLIC_PRIORITY;
    priority_addr += irqno;
    writel(1, priority_addr);
}

void plic_disable_irq(int irqno)
{
    if (irqno < 0 || irqno >= IRQ_MAX_NR)
    {
        LOG_E("[IRQ] plic disable bad irq %d!", irqno);
    }

    int hart = rt_hw_cpu_id();

    uint32_t *enable_addr = (uint32_t *)PLIC_SENABLE(hart);

    enable_addr += irqno / 32;
    uint8_t irqno_off = irqno % 32;
    // set uart's enable bit for this hart's S-mode.
    writel(readl((void *)enable_addr) & ~(1 << irqno_off), enable_addr);

    uint32_t *priority_addr = (uint32_t *) PLIC_PRIORITY;
    priority_addr += irqno;
    writel(0, priority_addr);
}

// ask the PLIC what interrupt we should serve.
int plic_claim(void)
{
    int hart = rt_hw_cpu_id();
    int irq = readl((void *)PLIC_SCLAIM(hart));
    return irq;
}

// tell the PLIC we've served this IRQ.
void plic_complete(int irq)
{
    int hart = rt_hw_cpu_id();
    writel(irq, (void *)PLIC_SCLAIM(hart));
}
