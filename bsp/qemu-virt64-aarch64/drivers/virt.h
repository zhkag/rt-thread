/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-17     GuEe-GUI     the first version
 */

#ifndef VIRT_H__
#define VIRT_H__

#include <rtdef.h>

#ifdef RT_USING_LWP
#include <mmu.h>
#include <ioremap.h>

extern rt_mmu_info mmu_info;
#endif

#define __REG32(x)          (*((volatile unsigned int *)(x)))
#define __REG16(x)          (*((volatile unsigned short *)(x)))

/* UART */
#define PL011_UART0_BASE    0x09000000
#define PL011_UART0_SIZE    0x00001000
#define PL011_UART0_IRQNUM  (32 + 1)

/* RTC */
#define PL031_RTC_BASE      0x9010000
#define PL031_RTC_SIZE      0x00001000
#define PL031_RTC_IRQNUM    (32 + 2)

/* VirtIO */
#define VIRTIO_MMIO_BASE    0x0a000000
#define VIRTIO_MMIO_SIZE    0x00000200
#define VIRTIO_MAX_NR       32
#define VIRTIO_IRQ_BASE     (32 + 16)
#define VIRTIO_VENDOR_ID    0x554d4551  /* "QEMU" */

/* GIC */
#define MAX_HANDLERS        96
#define GIC_IRQ_START       0
#define ARM_GIC_NR_IRQS     96
#define ARM_GIC_MAX_NR      1

#define IRQ_ARM_IPI_KICK    0
#define IRQ_ARM_IPI_CALL    1

/* GICv2 */
#define GIC_PL390_DISTRIBUTOR_PPTR      0x08000000
#define GIC_PL390_CONTROLLER_PPTR       0x08010000
#define GIC_PL390_HYPERVISOR_BASE       0x08030000
#define GIC_PL390_VIRTUAL_CPU_BASE      0x08040000

/* the basic constants and interfaces needed by gic */
rt_inline rt_ubase_t platform_get_gic_dist_base(void)
{
    return GIC_PL390_DISTRIBUTOR_PPTR;
}

rt_inline rt_ubase_t platform_get_gic_cpu_base(void)
{
    return GIC_PL390_CONTROLLER_PPTR;
}

#endif
