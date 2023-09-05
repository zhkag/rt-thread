/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021/1/30      lizirui      first version
 * 2021/10/20     JasonHu      move to trap.c
 */

#include <rthw.h>
#include <rtthread.h>
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "board.h"
#include "tick.h"

#include "drv_uart.h"
#include "encoding.h"
#include "stack.h"
#include "sbi.h"
#include "riscv.h"

#include "rt_interrupt.h"
#include "plic.h"

#ifdef RT_USING_USERSPACE
    #include "riscv_mmu.h"
    #include "mmu.h"
    #include "page.h"
    #include "lwp_arch.h"
#endif

void dump_regs(struct rt_hw_stack_frame *regs)
{
    rt_kprintf("--------------Dump Registers-----------------\n");

    rt_kprintf("Function Registers:\n");
    rt_kprintf("\tra(x1) = 0x%p(",regs->ra);
    rt_kprintf(")\n");
    rt_kprintf("\tuser_sp(x2) = 0x%p(",regs->user_sp_exc_stack);
    rt_kprintf(")\n");
    rt_kprintf("\tgp(x3) = 0x%p(",regs->gp);
    rt_kprintf(")\n");
    rt_kprintf("\ttp(x4) = 0x%p(",regs->tp);
    rt_kprintf(")\n");
    rt_kprintf("Temporary Registers:\n");
    rt_kprintf("\tt0(x5) = 0x%p(",regs->t0);
    rt_kprintf(")\n");
    rt_kprintf("\tt1(x6) = 0x%p(",regs->t1);
    rt_kprintf(")\n");
    rt_kprintf("\tt2(x7) = 0x%p(",regs->t2);
    rt_kprintf(")\n");
    rt_kprintf("\tt3(x28) = 0x%p(",regs->t3);
    rt_kprintf(")\n");
    rt_kprintf("\tt4(x29) = 0x%p(",regs->t4);
    rt_kprintf(")\n");
    rt_kprintf("\tt5(x30) = 0x%p(",regs->t5);
    rt_kprintf(")\n");
    rt_kprintf("\tt6(x31) = 0x%p(",regs->t6);
    rt_kprintf(")\n");
    rt_kprintf("Saved Registers:\n");
    rt_kprintf("\ts0/fp(x8) = 0x%p(",regs->s0_fp);
    rt_kprintf(")\n");
    rt_kprintf("\ts1(x9) = 0x%p(",regs->s1);
    rt_kprintf(")\n");
    rt_kprintf("\ts2(x18) = 0x%p(",regs->s2);
    rt_kprintf(")\n");
    rt_kprintf("\ts3(x19) = 0x%p(",regs->s3);
    rt_kprintf(")\n");
    rt_kprintf("\ts4(x20) = 0x%p(",regs->s4);
    rt_kprintf(")\n");
    rt_kprintf("\ts5(x21) = 0x%p(",regs->s5);
    rt_kprintf(")\n");
    rt_kprintf("\ts6(x22) = 0x%p(",regs->s6);
    rt_kprintf(")\n");
    rt_kprintf("\ts7(x23) = 0x%p(",regs->s7);
    rt_kprintf(")\n");
    rt_kprintf("\ts8(x24) = 0x%p(",regs->s8);
    rt_kprintf(")\n");
    rt_kprintf("\ts9(x25) = 0x%p(",regs->s9);
    rt_kprintf(")\n");
    rt_kprintf("\ts10(x26) = 0x%p(",regs->s10);
    rt_kprintf(")\n");
    rt_kprintf("\ts11(x27) = 0x%p(",regs->s11);
    rt_kprintf(")\n");
    rt_kprintf("Function Arguments Registers:\n");
    rt_kprintf("\ta0(x10) = 0x%p(",regs->a0);
    rt_kprintf(")\n");
    rt_kprintf("\ta1(x11) = 0x%p(",regs->a1);
    rt_kprintf(")\n");
    rt_kprintf("\ta2(x12) = 0x%p(",regs->a2);
    rt_kprintf(")\n");
    rt_kprintf("\ta3(x13) = 0x%p(",regs->a3);
    rt_kprintf(")\n");
    rt_kprintf("\ta4(x14) = 0x%p(",regs->a4);
    rt_kprintf(")\n");
    rt_kprintf("\ta5(x15) = 0x%p(",regs->a5);
    rt_kprintf(")\n");
    rt_kprintf("\ta6(x16) = 0x%p(",regs->a6);
    rt_kprintf(")\n");
    rt_kprintf("\ta7(x17) = 0x%p(",regs->a7);
    rt_kprintf(")\n");
    rt_kprintf("sstatus = 0x%p\n",regs->sstatus);
    rt_kprintf("\t%s\n",(regs->sstatus & SSTATUS_SIE) ? "Supervisor Interrupt Enabled" : "Supervisor Interrupt Disabled");
    rt_kprintf("\t%s\n",(regs->sstatus & SSTATUS_SPIE) ? "Last Time Supervisor Interrupt Enabled" : "Last Time Supervisor Interrupt Disabled");
    rt_kprintf("\t%s\n",(regs->sstatus & SSTATUS_SPP) ? "Last Privilege is Supervisor Mode" : "Last Privilege is User Mode");
    rt_kprintf("\t%s\n",(regs->sstatus & SSTATUS_PUM) ? "Permit to Access User Page" : "Not Permit to Access User Page");
    rt_kprintf("\t%s\n",(regs->sstatus & (1 << 19)) ? "Permit to Read Executable-only Page" : "Not Permit to Read Executable-only Page");
    rt_size_t satp_v = read_csr(satp);
    rt_kprintf("satp = 0x%p\n",satp_v);

#ifdef RT_USING_USERSPACE
    rt_kprintf("\tCurrent Page Table(Physical) = 0x%p\n",__MASKVALUE(satp_v,__MASK(44)) << PAGE_OFFSET_BIT);
    rt_kprintf("\tCurrent ASID = 0x%p\n",__MASKVALUE(satp_v >> 44,__MASK(16)) << PAGE_OFFSET_BIT);
#endif

    const char *mode_str = "Unknown Address Translation/Protection Mode";

    switch(__MASKVALUE(satp_v >> 60,__MASK(4)))
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

    rt_kprintf("\tMode = %s\n",mode_str);
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
                                    "Store/AMO Page Fault"
                                };

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

extern struct rt_irq_desc isr_table[];

void generic_handle_irq(int irq)
{
    rt_isr_handler_t isr;
    void *param;

    if (irq < 0 || irq >= IRQ_MAX_NR)
    {
        LOG_E("bad irq number %d!\n", irq);
        return;
    }

    if (!irq)   // irq = 0 => no irq
    {
        LOG_W("no irq!\n");
        return;
    }
    isr = isr_table[IRQ_OFFSET + irq].handler;
    param = isr_table[IRQ_OFFSET + irq].param;
    if (isr != RT_NULL)
    {
        isr(irq, param);
    }
    /* complete irq. */
    plic_complete(irq);
}

static const char *get_exception_msg(int id)
{
    const char *msg;
    if (id < sizeof(Exception_Name) / sizeof(const char *))
    {
        msg = Exception_Name[id];
    }
    else
    {
        msg = "Unknown Exception";
    }
    return msg;
}

void handle_user(rt_size_t scause, rt_size_t stval, rt_size_t sepc, struct rt_hw_stack_frame *sp)
{
    rt_size_t id = __MASKVALUE(scause, __MASK(63UL));

#ifdef RT_USING_USERSPACE
    /* user page fault */
    if (id == EP_LOAD_PAGE_FAULT ||
        id == EP_STORE_PAGE_FAULT)
    {
        if (arch_expand_user_stack((void *)stval))
        {
            return;
        }
    }
#endif
    LOG_E("[FATAL ERROR] Exception %ld:%s", id, get_exception_msg(id));
    LOG_E("scause:0x%p,stval:0x%p,sepc:0x%p", scause, stval, sepc);
    dump_regs(sp);

    rt_hw_backtrace((uint32_t *)sp->s0_fp, sepc);

    LOG_E("User Fault, killing thread: %s", rt_thread_self()->name);
    sys_exit(-1);
}

static void vector_enable(struct rt_hw_stack_frame *sp)
{
    sp->sstatus |= SSTATUS_VS_INITIAL;
}

/**
 * we use quick method to detect V / D support, and do not distinguish V/D instruction
 */
static int illegal_inst_recoverable(rt_ubase_t stval, struct rt_hw_stack_frame *sp)
{
    // first 7 bits is opcode
    int opcode = stval & 0x7f;
    int csr = (stval & 0xFFF00000) >> 20;
    // ref riscv-v-spec-1.0, [Vector Instruction Formats]
    int width = ((stval & 0x7000) >> 12) - 1;
    int flag = 0;

    switch (opcode)
    {
    case 0x57: // V
    case 0x27: // scalar FLOAT
    case 0x07:
    case 0x73: // CSR
        flag = 1;
        break;
    }

    if (flag)
    {
        vector_enable(sp);
    }

    return flag;
}

#define IN_USER_SPACE (stval >= USER_VADDR_START && stval < USER_VADDR_TOP)
#define PAGE_FAULT (id == EP_LOAD_PAGE_FAULT || id == EP_STORE_PAGE_FAULT)

#ifdef RT_USING_USAGE
static volatile uint64_t time_elapsed = 0;

static uint64_t get_ticks()
{
    __asm__ __volatile__(
        "rdtime %0"
        : "=r"(time_elapsed));
    return time_elapsed;
}
#endif

/* Trap entry */
void handle_trap(rt_size_t scause,rt_size_t stval,rt_size_t sepc,struct rt_hw_stack_frame *sp)
{
    rt_size_t id = __MASKVALUE(scause,__MASK(63UL));
    const char *msg;

    /* supervisor external interrupt */
    if ((SCAUSE_INTERRUPT & scause) && SCAUSE_S_EXTERNAL_INTR == (scause & 0xff))
    {
#ifdef RT_USING_USAGE_ROMOVE_TRAP_TIME
        rt_thread_t current_thread;
        volatile uint64_t enter_trap;
        volatile uint64_t leave_trap;
        current_thread = rt_thread_self();
        enter_trap = get_ticks();
        rt_interrupt_enter();
        plic_handle_irq();
        leave_trap = get_ticks();
        if(current_thread->user_data > (leave_trap - enter_trap))
            current_thread->user_data -= (leave_trap - enter_trap);
        rt_interrupt_leave();
#else
        rt_interrupt_enter();
        plic_handle_irq();
        rt_interrupt_leave();
#endif
        return;
    }
    else if ((SCAUSE_INTERRUPT | SCAUSE_S_TIMER_INTR) == scause)
    {
        /* supervisor timer */
        rt_interrupt_enter();
        tick_isr();
        rt_interrupt_leave();
        return;
    }
    else if (SCAUSE_INTERRUPT & scause)
    {
        if(id < sizeof(Interrupt_Name) / sizeof(const char *))
        {
            msg = Interrupt_Name[id];
        }
        else
        {
            msg = "Unknown Interrupt";
        }
        LOG_E("Unhandled Interrupt %ld:%s\n",id,msg);
    }
    else
    {
        if (scause == 0x2)
        {
            if (!(sp->sstatus & SSTATUS_VS) && illegal_inst_recoverable(stval, sp))
                return;
        }
        if (!(sp->sstatus & 0x100) || (PAGE_FAULT && IN_USER_SPACE))
        {
            handle_user(scause, stval, sepc, sp);
            // after handle_user(), return to user space.
            // otherwise it never returns
            return ;
        }

        // handle kernel exception:
        rt_kprintf("Unhandled Exception %ld:%s\n", id, get_exception_msg(id));
    }

    rt_kprintf("scause:0x%p,stval:0x%p,sepc:0x%p\n", scause, stval, sepc);
    dump_regs(sp);
    rt_kprintf("--------------Thread list--------------\n");
    rt_kprintf("current thread: %s\n", rt_thread_self()->name);
    list_process();

    extern struct rt_thread *rt_current_thread;
    rt_kprintf("--------------Backtrace--------------\n");
    rt_hw_backtrace((uint32_t *)sp->s0_fp, sepc);

    while (1)
        ;
}
