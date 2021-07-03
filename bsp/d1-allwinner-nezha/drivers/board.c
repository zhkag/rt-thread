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
#include "stack.h"

#ifdef RT_USING_USERSPACE
    #include "riscv_mmu.h"
    #include "mmu.h"
    #include "page.h"
    #include "lwp_arch.h"

    //这个结构体描述了buddy system的页分配范围
    rt_region_t init_page_region =
    {
        (rt_size_t)RT_HW_PAGE_START,
        (rt_size_t)RT_HW_PAGE_END
    };

    //内核页表
    volatile rt_size_t MMUTable[__SIZE(VPN2_BIT)] __attribute__((aligned(4 * 1024)));
    rt_mmu_info mmu_info;

#endif

//初始化BSS节区
void init_bss(void)
{
    unsigned int *dst;

    dst = &__bss_start;
    while (dst < &__bss_end)
    {
        *dst++ = 0;
    }
}

static void __rt_assert_handler(const char *ex_string, const char *func, rt_size_t line)
{
    rt_kprintf("(%s) assertion failed at function:%s, line number:%d \n", ex_string, func, line);
    asm volatile("ebreak":::"memory");
}

//BSP的C入口
void primary_cpu_entry(void)
{
    extern void entry(void);

    //初始化BSS
    init_bss();
    //关中断
    rt_hw_interrupt_disable();
    rt_assert_set_hook(__rt_assert_handler);
    //启动RT-Thread Smart内核
    entry();
}

//中断初始化程序
void rt_hw_interrupt_init()
{
    /* Enable machine external interrupts. */
    set_csr(sie, SIP_SEIP);
}

//这个初始化程序由内核主动调用，此时调度器还未启动，因此在此不能使用依赖线程上下文的函数
void rt_hw_board_init(void)
{
    /* initalize interrupt */
    rt_hw_interrupt_init();
    /* initialize hardware interrupt */
    rt_hw_uart_init();
    rt_hw_tick_init();
    #ifdef RT_USING_HEAP
        /* initialize memory system */
        rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
    #endif

    #ifdef RT_USING_CONSOLE
        /* set console device */
        rt_console_set_device("uart");
    #endif /* RT_USING_CONSOLE */

    #ifdef RT_USING_COMPONENTS_INIT
        rt_components_board_init();
    #endif
    #ifdef RT_USING_HEAP
        rt_kprintf("heap: [0x%08x - 0x%08x]\n", (rt_ubase_t) RT_HW_HEAP_BEGIN, (rt_ubase_t) RT_HW_HEAP_END);
    #endif
    
    #ifdef RT_USING_USERSPACE
        rt_hw_mmu_map_init(&mmu_info,(void *)0x100000000UL,0xFFFFFFFEFFFFFFFFUL,(rt_size_t *)MMUTable,0);
        rt_page_init(init_page_region);
        rt_hw_mmu_kernel_map_init(&mmu_info,0x00000000UL,0xFFFFFFFFUL);
        //将低1GB MMIO区域设置为无Cache与Strong Order访存模式
        MMUTable[0] &= ~PTE_CACHE;
        MMUTable[0] &= ~PTE_SHARE;
        MMUTable[0] |= PTE_SO;
        switch_mmu((void *)MMUTable);
    #endif
}

void rt_hw_cpu_reset(void)
{
    SBI_CALL_0(SBI_SHUTDOWN);
    while(1);
}
MSH_CMD_EXPORT_ALIAS(rt_hw_cpu_reset, reboot, reset machine);

#include "symbol_analysis.h"
void dump_regs(struct rt_hw_stack_frame *regs)
{
    rt_kprintf("--------------Dump Registers-----------------\n");

    rt_kprintf("Function Registers:\n");
    rt_kprintf("\tra(x1) = 0x%p(",regs -> ra);print_symbol_info(regs -> ra,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\tuser_sp(x2) = 0x%p(",regs -> user_sp_exc_stack);print_symbol_info(regs -> user_sp_exc_stack,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\tgp(x3) = 0x%p(",regs -> gp);print_symbol_info(regs -> gp,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ttp(x4) = 0x%p(",regs -> tp);print_symbol_info(regs -> tp,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("Temporary Registers:\n");
    rt_kprintf("\tt0(x5) = 0x%p(",regs -> t0);print_symbol_info(regs -> t0,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\tt1(x6) = 0x%p(",regs -> t1);print_symbol_info(regs -> t1,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\tt2(x7) = 0x%p(",regs -> t2);print_symbol_info(regs -> t2,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\tt3(x28) = 0x%p(",regs -> t3);print_symbol_info(regs -> t3,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\tt4(x29) = 0x%p(",regs -> t4);print_symbol_info(regs -> t4,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\tt5(x30) = 0x%p(",regs -> t5);print_symbol_info(regs -> t5,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\tt6(x31) = 0x%p(",regs -> t6);print_symbol_info(regs -> t6,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("Saved Registers:\n");
    rt_kprintf("\ts0/fp(x8) = 0x%p(",regs -> s0_fp);print_symbol_info(regs -> s0_fp,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ts1(x9) = 0x%p(",regs -> s1);print_symbol_info(regs -> s1,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ts2(x18) = 0x%p(",regs -> s2);print_symbol_info(regs -> s2,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ts3(x19) = 0x%p(",regs -> s3);print_symbol_info(regs -> s3,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ts4(x20) = 0x%p(",regs -> s4);print_symbol_info(regs -> s4,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ts5(x21) = 0x%p(",regs -> s5);print_symbol_info(regs -> s5,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ts6(x22) = 0x%p(",regs -> s6);print_symbol_info(regs -> s6,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ts7(x23) = 0x%p(",regs -> s7);print_symbol_info(regs -> s7,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ts8(x24) = 0x%p(",regs -> s8);print_symbol_info(regs -> s8,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ts9(x25) = 0x%p(",regs -> s9);print_symbol_info(regs -> s9,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ts10(x26) = 0x%p(",regs -> s10);print_symbol_info(regs -> s10,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ts11(x27) = 0x%p(",regs -> s11);print_symbol_info(regs -> s11,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("Function Arguments Registers:\n");
    rt_kprintf("\ta0(x10) = 0x%p(",regs -> a0);print_symbol_info(regs -> a0,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ta1(x11) = 0x%p(",regs -> a1);print_symbol_info(regs -> a1,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ta2(x12) = 0x%p(",regs -> a2);print_symbol_info(regs -> a2,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ta3(x13) = 0x%p(",regs -> a3);print_symbol_info(regs -> a3,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ta4(x14) = 0x%p(",regs -> a4);print_symbol_info(regs -> a4,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ta5(x15) = 0x%p(",regs -> a5);print_symbol_info(regs -> a5,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ta6(x16) = 0x%p(",regs -> a6);print_symbol_info(regs -> a6,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("\ta7(x17) = 0x%p(",regs -> a7);print_symbol_info(regs -> a7,RT_FALSE);rt_kprintf(")\n");
    rt_kprintf("sstatus = 0x%p\n",regs -> sstatus);
    rt_kprintf("\t%s\n",(regs -> sstatus & SSTATUS_SIE) ? "Supervisor Interrupt Enabled" : "Supervisor Interrupt Disabled");
    rt_kprintf("\t%s\n",(regs -> sstatus & SSTATUS_SPIE) ? "Last Time Supervisor Interrupt Enabled" : "Last Time Supervisor Interrupt Disabled");
    rt_kprintf("\t%s\n",(regs -> sstatus & SSTATUS_SPP) ? "Last Privilege is Supervisor Mode" : "Last Privilege is User Mode");
    rt_kprintf("\t%s\n",(regs -> sstatus & SSTATUS_PUM) ? "Permit to Access User Page" : "Not Permit to Access User Page");
    rt_kprintf("\t%s\n",(regs -> sstatus & (1 << 19)) ? "Permit to Read Executable-only Page" : "Not Permit to Read Executable-only Page");
    rt_size_t satp_v = read_csr(satp);
    rt_kprintf("satp = 0x%p\n",satp_v);
    rt_kprintf("\tCurrent Page Table(Physical) = 0x%p\n",__MASKVALUE(satp_v,__MASK(44)) << PAGE_OFFSET_BIT);
    rt_kprintf("\tCurrent ASID = 0x%p\n",__MASKVALUE(satp_v >> 44,__MASK(16)) << PAGE_OFFSET_BIT);
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
    print_stacktrace(regs -> epc,regs -> s0_fp);
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

//Trap处理入口
void handle_trap(rt_size_t scause,rt_size_t stval,rt_size_t sepc,struct rt_hw_stack_frame *sp)
{
    // rt_kprintf(".");
    if(scause == (uint64_t)(0x8000000000000005))//S态时钟中断
    {
        rt_interrupt_enter();
        tick_isr();
        rt_interrupt_leave();
    }
    /*else if(scause == (uint64_t)(0x8000000000000009))
    {
        rt_kprintf("a\n");
        while(1);
        extern struct rt_serial_device  serial1;
        rt_hw_serial_isr(&serial1,RT_SERIAL_EVENT_RX_IND);
    }*/
    else
    {
        rt_size_t id = __MASKVALUE(scause,__MASK(63UL));
        const char *msg;

        //若为中断，则认为这是一个未处理的中断
        if(scause >> 63)
        {
            if(id < sizeof(Interrupt_Name) / sizeof(const char *))
            {
                msg = Interrupt_Name[id];
            }
            else
            {
                msg = "Unknown Interrupt";
            }

            rt_kprintf("Unhandled Interrupt %ld:%s\n",id,msg);
        }
        else//否则是一个异常，需要特殊处理缺页异常
        {
            #ifdef RT_USING_USERSPACE
                if(id == 15)//若为缺页异常，则进行用户栈扩展
                {
                    arch_expand_user_stack((void *)stval);
                    return;
                }
            #endif

            if(id < sizeof(Exception_Name) / sizeof(const char *))
            {
                msg = Exception_Name[id];
            }
            else
            {
                msg = "Unknown Exception";
            }

            rt_kprintf("Unhandled Exception %ld:%s\n",id,msg);
        }

        rt_kprintf("scause:0x%p,stval:0x%p,sepc:0x%p\n",scause,stval,sepc);
        dump_regs(sp);
        while(1);
    }
}
