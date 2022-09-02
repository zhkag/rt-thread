/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     lizhirui     first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "board.h"
#include "tick.h"

#include "drv_uart.h"
#include "encoding.h"
#include "stack.h"
#include "sbi.h"
#include "riscv.h"
#include "plic.h"
#include "stack.h"

#ifdef RT_USING_USERSPACE
#include "riscv_mmu.h"
#include "mmu.h"
#include "page.h"
#include "lwp_arch.h"

rt_region_t init_page_region = {(rt_size_t)RT_HW_PAGE_START, (rt_size_t)RT_HW_PAGE_END};

rt_mmu_info mmu_info;

extern size_t MMUTable[];

struct mem_desc platform_mem_desc[] = {
    {KERNEL_VADDR_START, KERNEL_VADDR_START + 0x10000000 - 1, KERNEL_VADDR_START + PV_OFFSET, NORMAL_MEM},
    };

#define NUM_MEM_DESC (sizeof(platform_mem_desc) / sizeof(platform_mem_desc[0]))

#endif

void init_bss(void)
{
    unsigned int *dst;

    dst = &__bss_start;
    while (dst < &__bss_end)
    {
        *dst++ = 0;
    }
}

void primary_cpu_entry(void)
{
    extern void entry(void);

    /* disable global interrupt */
    init_bss();
    rt_hw_interrupt_disable();
    entry();
}

#define IOREMAP_SIZE (1ul << 30)

void rt_hw_board_init(void)
{
#ifdef RT_USING_USERSPACE
    rt_page_init(init_page_region);
    /* init mmu_info structure */
    rt_hw_mmu_map_init(&mmu_info, (void *)(USER_VADDR_START - IOREMAP_SIZE), IOREMAP_SIZE, (rt_size_t *)MMUTable, 0);
    // this API is reserved currently since PLIC etc had not been porting completely to MMU version
    rt_hw_mmu_kernel_map_init(&mmu_info, 0x00000000UL, 0x80000000);
    /* setup region, and enable MMU */
    rt_hw_mmu_setup(&mmu_info, platform_mem_desc, NUM_MEM_DESC);

#endif

#ifdef RT_USING_HEAP
    /* initialize memory system */
    rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
#endif

    rt_hw_uart_init();

#ifdef RT_USING_CONSOLE
    /* set console device */
    rt_console_set_device("uart");

#ifdef RT_USING_HEAP
    rt_kprintf("heap: [0x%08x - 0x%08x]\n", (rt_ubase_t)RT_HW_HEAP_BEGIN, (rt_ubase_t)RT_HW_HEAP_END);
#endif

    rt_hw_interrupt_init();

    rt_hw_tick_init();

#endif /* RT_USING_CONSOLE */

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

    plic_init();
}

void rt_hw_cpu_reset(void)
{
    sbi_shutdown();

    while (1)
        ;
}
MSH_CMD_EXPORT_ALIAS(rt_hw_cpu_reset, reboot, reset machine);

void dump_regs(struct rt_hw_stack_frame *regs)
{
    rt_kprintf("--------------Dump Registers-----------------\n");

    rt_kprintf("Function Registers:\n");
    rt_kprintf("\tra(x1) = 0x%p\tuser_sp = 0x%p\n", regs->ra, regs->user_sp_exc_stack);
    rt_kprintf("\tgp(x3) = 0x%p\ttp(x4) = 0x%p\n", regs->gp, regs->tp);
    rt_kprintf("Temporary Registers:\n");
    rt_kprintf("\tt0(x5) = 0x%p\tt1(x6) = 0x%p\n", regs->t0, regs->t1);
    rt_kprintf("\tt2(x7) = 0x%p\n", regs->t2);
    rt_kprintf("\tt3(x28) = 0x%p\tt4(x29) = 0x%p\n", regs->t3, regs->t4);
    rt_kprintf("\tt5(x30) = 0x%p\tt6(x31) = 0x%p\n", regs->t5, regs->t6);
    rt_kprintf("Saved Registers:\n");
    rt_kprintf("\ts0/fp(x8) = 0x%p\ts1(x9) = 0x%p\n", regs->s0_fp, regs->s1);
    rt_kprintf("\ts2(x18) = 0x%p\ts3(x19) = 0x%p\n", regs->s2, regs->s3);
    rt_kprintf("\ts4(x20) = 0x%p\ts5(x21) = 0x%p\n", regs->s4, regs->s5);
    rt_kprintf("\ts6(x22) = 0x%p\ts7(x23) = 0x%p\n", regs->s6, regs->s7);
    rt_kprintf("\ts8(x24) = 0x%p\ts9(x25) = 0x%p\n", regs->s8, regs->s9);
    rt_kprintf("\ts10(x26) = 0x%p\ts11(x27) = 0x%p\n", regs->s10, regs->s11);
    rt_kprintf("Function Arguments Registers:\n");
    rt_kprintf("\ta0(x10) = 0x%p\ta1(x11) = 0x%p\n", regs->a0, regs->a1);
    rt_kprintf("\ta2(x12) = 0x%p\ta3(x13) = 0x%p\n", regs->a2, regs->a3);
    rt_kprintf("\ta4(x14) = 0x%p\ta5(x15) = 0x%p\n", regs->a4, regs->a5);
    rt_kprintf("\ta6(x16) = 0x%p\ta7(x17) = 0x%p\n", regs->a6, regs->a7);
    rt_kprintf("sstatus = 0x%p\n", regs->sstatus);
    rt_kprintf("\t%s\n", (regs->sstatus & SSTATUS_SIE) ? "Supervisor Interrupt Enabled" : "Supervisor Interrupt Disabled");
    rt_kprintf("\t%s\n", (regs->sstatus & SSTATUS_SPIE) ? "Last Time Supervisor Interrupt Enabled" : "Last Time Supervisor Interrupt Disabled");
    rt_kprintf("\t%s\n", (regs->sstatus & SSTATUS_SPP) ? "Last Privilege is Supervisor Mode" : "Last Privilege is User Mode");
    rt_kprintf("\t%s\n", (regs->sstatus & SSTATUS_PUM) ? "Permit to Access User Page" : "Not Permit to Access User Page");
    rt_kprintf("\t%s\n", (regs->sstatus & (1 << 19)) ? "Permit to Read Executable-only Page" : "Not Permit to Read Executable-only Page");
    rt_size_t satp_v = read_csr(satp);
    rt_kprintf("satp = 0x%p\n", satp_v);
    rt_kprintf("\tCurrent Page Table(Physical) = 0x%p\n", __MASKVALUE(satp_v, __MASK(44)) << PAGE_OFFSET_BIT);
    rt_kprintf("\tCurrent ASID = 0x%p\n", __MASKVALUE(satp_v >> 44, __MASK(16)) << PAGE_OFFSET_BIT);
    const char *mode_str = "Unknown Address Translation/Protection Mode";

    switch (__MASKVALUE(satp_v >> 60, __MASK(4)))
    {
    case 0:
        mode_str = "No Address Translation/Protection Mode";
        break;

    case 8:
        mode_str = "Page-based 39-bit Virtual Addressing Mode";
        break;

    case 9:
        mode_str = "Page-based 48-bit Virtual Addressing Mode";
        break;
    }

    rt_kprintf("\tMode = %s\n", mode_str);
    rt_kprintf("-----------------Dump OK---------------------\n");
}

static const char *Exception_Name[] =
    {
        "Instruction Address Misaligned",
        "Instruction Access Fault",
        "Illegal Instruction",
        "Breakpoint",
        "Load Address Misaligned",
        "Load Access Fault",
        "Store/AMO Address Misaligned",
        "Store/AMO Access Fault",
        "Environment call from U-mode",
        "Environment call from S-mode",
        "Reserved-10",
        "Reserved-11",
        "Instruction Page Fault",
        "Load Page Fault",
        "Reserved-14",
        "Store/AMO Page Fault"};

static const char *Interrupt_Name[] =
    {
        "User Software Interrupt",
        "Supervisor Software Interrupt",
        "Reversed-2",
        "Reversed-3",
        "User Timer Interrupt",
        "Supervisor Timer Interrupt",
        "Reversed-6",
        "Reversed-7",
        "User External Interrupt",
        "Supervisor External Interrupt",
        "Reserved-10",
        "Reserved-11",
};

extern struct rt_irq_desc irq_desc[];

void handle_trap(rt_size_t scause, rt_size_t stval, rt_size_t sepc, struct rt_hw_stack_frame *sp)
{
    // rt_kprintf(".");
    if (scause == (uint64_t)(0x8000000000000005))
    {
        rt_interrupt_enter();
        tick_isr();
        rt_interrupt_leave();
    }
    else if (scause == (uint64_t)(0x8000000000000009))
    {
        int plic_irq = plic_claim();
        plic_complete(plic_irq);
        irq_desc[plic_irq].handler(plic_irq, irq_desc[plic_irq].param);
    }
    else
    {
        rt_size_t id = __MASKVALUE(scause, __MASK(63UL));
        const char *msg;

        if (scause >> 63)
        {
            if (id < sizeof(Interrupt_Name) / sizeof(const char *))
            {
                msg = Interrupt_Name[id];
            }
            else
            {
                msg = "Unknown Interrupt";
            }

            rt_kprintf("Unhandled Interrupt %ld:%s\n", id, msg);
        }
        else
        {
#ifdef RT_USING_USERSPACE
            if (id == 15)
            {
                arch_expand_user_stack((void *)stval);
                return;
            }
#endif

            if (id < sizeof(Exception_Name) / sizeof(const char *))
            {
                msg = Exception_Name[id];
            }
            else
            {
                msg = "Unknown Exception";
            }

            rt_kprintf("Unhandled Exception %ld:%s\n", id, msg);
        }

        rt_kprintf("scause:0x%p,stval:0x%p,sepc:0x%p\n", scause, stval, sepc);
        dump_regs(sp);
        while (1)
            ;
    }
}
