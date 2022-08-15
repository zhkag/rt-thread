/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */
#ifndef  __RT_HW_CPU_H__
#define  __RT_HW_CPU_H__

#include <rthw.h>
#include <rtthread.h>
#include <stdbool.h>

struct cpu_ops_t {
    const char *method;
    int     (*cpu_init)(rt_uint32_t id);
    int     (*cpu_boot)(rt_uint32_t id);
	void    (*cpu_shutdown)(void);
};

extern rt_uint64_t rt_cpu_mpidr_early[];

#define cpuid_to_hwid(cpuid) rt_cpu_mpidr_early[cpuid]

extern void rt_hw_cpu_shutdown(void);

extern int rt_hw_cpu_init();

extern int rt_hw_cpu_boot_secondary(int num_cpus, rt_uint64_t *cpu_hw_ids, struct cpu_ops_t *cpu_ops[]);

extern struct cpu_ops_t cpu_ops_psci;

extern struct cpu_ops_t cpu_ops_spin_tbl;

#endif /* __RT_HW_CPU_H__ */