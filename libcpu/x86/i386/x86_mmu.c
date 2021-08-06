/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-17     JasonHu      first version
 */

#include <rthw.h>
#include <rtthread.h>

#include "mmu.h"
#include <i386.h>

void mmu_set_pagetable(rt_ubase_t addr)
{
    /* set new pgdir will flush tlb */
    write_cr3(addr);
}

void mmu_enable_user_page_access()
{
}

void mmu_disable_user_page_access()
{
}

void mmu_enable()
{
    write_cr0(read_cr0() | CR0_PG);
}
