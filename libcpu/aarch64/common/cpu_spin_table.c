/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */
#include <rthw.h>
#include <rtthread.h>
#include <ioremap.h>
#include "cpu.h"

#ifdef RT_USING_FDT
#include <dtb_node.h>
#include "entry_point.h"

#define get_cpu_node(cpuid) _cpu_node[cpuid]
extern struct dtb_node *_cpu_node[];

static rt_uint64_t cpu_release_addr[RT_CPUS_NR];

static int spin_table_cpu_init(rt_uint32_t cpuid)
{
    struct dtb_node *cpu = get_cpu_node(cpuid);
    if (!cpu)
        return -1;
    
    int size;
    rt_uint64_t head = (rt_uint64_t)dtb_node_get_dtb_node_property_value(cpu, "cpu-release-addr", &size);

    cpu_release_addr[cpuid] = fdt64_to_cpu(head);

    return 0;
}

static int spin_table_cpu_boot(rt_uint32_t cpuid)
{
    rt_uint64_t secondary_entry_pa = (rt_uint64_t)_secondary_cpu_entry + PV_OFFSET;
    // map release_addr to addressable place
    void *rel_va = rt_ioremap((void *)cpu_release_addr[cpuid], sizeof(cpu_release_addr[0]));

    if (!rel_va)
        return -1;

    __asm__ volatile ("str %0, [%1]"::"rZ"(secondary_entry_pa), "r"(rel_va));
    __asm__ volatile ("dsb sy");
    __asm__ volatile ("sev");
    rt_iounmap(rel_va);
    return 0;
}
#endif /* RT_USING_FDT */

struct cpu_ops_t cpu_ops_spin_tbl = {
    .method = "spin-table",
#ifdef RT_USING_FDT
    .cpu_init = spin_table_cpu_init,
    .cpu_boot = spin_table_cpu_boot,
#endif
};
