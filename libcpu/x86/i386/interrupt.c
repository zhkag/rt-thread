/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-14     JasonHu      first version
 */

#include <rtthread.h>
#include <rthw.h>
#include <rtdbg.h>

#include <interrupt.h>
#include <stackframe.h>
#include <backtrace.h>
#include <pic.h>
#include <lwp_arch.h>

typedef void (*rt_hw_intr_handler_t)(rt_hw_stack_frame_t *);

static rt_hw_intr_handler_t interrupt_handlers[MAX_INTR_NR] = {0};
static struct rt_irq_desc irq_desc[MAX_IRQ_NR] = {0};

static char *hw_exception_names[] = {
    "#DE Divide Error",
    "#DB Debug Exception",
    "NMI Interrupt",
    "#BP Breakpoint Exception",
    "#OF Overflow Exception",
    "#BR BOUND Range Exceeded Exception",
    "#UD Invalid Opcode Exception",
    "#NM Device Not Available Exception",
    "#DF Double Fault Exception",
    "Coprocessor Segment Overrun",
    "#TS Invalid TSS Exception",
    "#NP Segment Not Present",
    "#SS Stack Fault Exception",
    "#GP General Protection Exception",
    "#PF Page-Fault Exception",
    "Reserved",
    "#MF x87 FPU Floating-Point Error",
    "#AC Alignment Check Exception",
    "#MC Machine-Check Exception",
    "#XF SIMD Floating-Point Exception",
    "Unknown Exception"
};

static void exception_frame_dump(rt_hw_stack_frame_t *frame);

static void rt_hw_interrupt_handle(int vector, void *param)
{
    rt_kprintf("UN-handled interrupt %d occurred!!!\n", vector);
}

static void hw_general_handler(rt_hw_stack_frame_t *frame)
{
    rt_kprintf("general intr %d handled\n", frame->vec_no);
}

static void hw_external_handler(rt_hw_stack_frame_t *frame)
{
    int irqno = frame->vec_no - IRQ_INTR_BASE;
    if (irqno < 0 || irqno >= MAX_IRQ_NR)
    {
        dbg_log(DBG_ERROR, "unknown IRQ %d occurred!!\n", irqno);
        return;
    }
    irq_desc[irqno].handler(irqno, irq_desc[irqno].param);
    rt_hw_pic_ack(irqno);
}


#ifdef RT_USING_LWP
static int check_user_stack(rt_hw_stack_frame_t *frame)
{
    if (frame->vec_no == EXCEPTION_PAGE_FAULT)
    {
        void *fault_addr = (void *)read_cr2();  // get page fault addr
        rt_interrupt_leave();
        if (arch_expand_user_stack(fault_addr))
        {
            rt_interrupt_enter();
            return 1;
        }
        rt_interrupt_enter();
    }
    return 0;
}
#endif  /* RT_USING_LWP */

static void hw_exception_handler(rt_hw_stack_frame_t *frame)
{
#ifdef RT_USING_LWP
    if (check_user_stack(frame))
        return;
#endif  /* RT_USING_LWP */
    rt_thread_t cur = rt_thread_self();
    rt_kprintf("thread name: %s\n", cur->name);

#ifdef RT_USING_LWP
    if (cur->lwp)
    {
        struct rt_lwp *lwp = cur->lwp;
        rt_kprintf("thread id:%d\n", lwp->pid);
    }
#endif  /* RT_USING_LWP */

    exception_frame_dump(frame);
    rt_hw_print_backtrace();
    /* unhandled exception */
    rt_hw_interrupt_disable();
    for (;;)
        ;
}

rt_base_t rt_hw_interrupt_disable(void)
{
    rt_base_t level;
    __asm__ __volatile__("pushfl ; popl %0 ; cli":"=g" (level): :"memory");
    return level;
}

void rt_hw_interrupt_enable(rt_base_t level)
{
    __asm__ __volatile__("pushl %0 ; popfl": :"g" (level):"memory", "cc");
}

void rt_hw_interrupt_dispatch(rt_hw_stack_frame_t *frame)
{
    rt_ubase_t vec_no = frame->vec_no;
    if (vec_no < 0 || vec_no >= MAX_INTR_NR)
    {
        dbg_log(DBG_ERROR, "unknown intr vector %x!\n", frame->vec_no);
        return;
    }
    interrupt_handlers[vec_no](frame);
}

void rt_hw_stack_frame_dump(rt_hw_stack_frame_t *frame)
{
    rt_kprintf("====stack frame dump====\n");
    rt_kprintf("edi:%x esi:%x ebp:%x esp dummy:%x ebx:%x edx:%x ecx:%x eax:%x\n",
        frame->edi, frame->esi, frame->ebp, frame->esp_dummy,
        frame->ebx, frame->edx, frame->ecx, frame->eax);
    rt_kprintf("gs:%x fs:%x es:%x ds:%x error code:%x eip:%x cs:%x eflags:%x esp:%x ss:%x\n",
        frame->gs, frame->fs, frame->es, frame->ds, frame->error_code,
        frame->eip, frame->cs, frame->eflags, frame->esp, frame->ss);
}

static void exception_frame_dump(rt_hw_stack_frame_t *frame)
{
    rt_kprintf("====exception frame dump====\n");
    rt_kprintf("Stack frame: exception name %s\n", hw_exception_names[frame->vec_no]);
    if (frame->vec_no == 14)
    {
        rt_kprintf("page fault addr: %p\n", read_cr2());
    }
    rt_hw_stack_frame_dump(frame);
    if (frame->error_code != 0xFFFFFFFF)
    {
        if (frame->error_code & 1)
        {
            rt_kprintf("    External Event: NMI,hard interruption,ect.\n");
        }
        else
        {
            rt_kprintf("    Not External Event: inside.\n");
        }
        if (frame->error_code & (1 << 1))
        {
            rt_kprintf("    IDT: selector in idt.\n");
        }
        else
        {
            rt_kprintf("    IDT: selector in gdt or ldt.\n");
        }
        if(frame->error_code & (1 <<2 ))
        {
            rt_kprintf("    TI: selector in ldt.\n");
        }
        else
        {
            rt_kprintf("    TI: selector in gdt.\n");
        }
        rt_kprintf("    Selector: idx %d\n", (frame->error_code&0xfff8)>>3);
    }
}

/**
 * This function will mask a interrupt.
 * @param vector the interrupt number
 */
void rt_hw_interrupt_mask(int vector)
{
    rt_hw_pic_disable(vector);
}

/**
 * This function will un-mask a interrupt.
 * @param vector the interrupt number
 */
void rt_hw_interrupt_umask(int vector)
{
    rt_hw_pic_enable(vector);
}

/**
 * This function will install a interrupt service routine to a interrupt.
 * @param vector the interrupt number
 * @param new_handler the interrupt service routine to be installed
 * @param old_handler the old interrupt service routine
 */
rt_isr_handler_t rt_hw_interrupt_install(int vector, rt_isr_handler_t handler,
        void *param, const char *name)
{
    rt_isr_handler_t old_handler = RT_NULL;

    if(vector < MAX_IRQ_NR)
    {
        old_handler = irq_desc[vector].handler;
        if (handler != RT_NULL)
        {
            irq_desc[vector].handler = (rt_isr_handler_t)handler;
            irq_desc[vector].param = param;
#ifdef RT_USING_INTERRUPT_INFO
            rt_snprintf(irq_desc[vector].name, RT_NAME_MAX - 1, "%s", name);
            irq_desc[vector].counter = 0;
#endif
        }
    }

    return old_handler;
}

extern volatile rt_ubase_t rt_interrupt_from_thread;
extern volatile rt_ubase_t rt_interrupt_to_thread;
extern volatile rt_ubase_t rt_thread_switch_interrupt_flag;
/**
 * This function will initialize hardware interrupt
 */
void rt_hw_interrupt_init(void)
{
    rt_interrupt_from_thread = 0;
    rt_interrupt_to_thread = 0;
    rt_thread_switch_interrupt_flag = 0;
    int i;
    for (i = 0; i < MAX_INTR_NR; i++)
    {
        if (i < IRQ_INTR_BASE)
        {
            interrupt_handlers[i] = hw_exception_handler;
        }
        else if (i >= IRQ_INTR_BASE && i < IRQ_INTR_BASE + MAX_IRQ_NR)
        {
            interrupt_handlers[i] = hw_external_handler;
        }
        else
        {
            interrupt_handlers[i] = hw_general_handler;
        }
    }
    for (i = 0; i < MAX_IRQ_NR; i++)
    {
        irq_desc[i].handler = rt_hw_interrupt_handle;
        irq_desc[i].param = RT_NULL;
#ifdef RT_USING_INTERRUPT_INFO
        rt_snprintf(irq_desc[i].name, RT_NAME_MAX - 1, "default");
        irq_desc[i].counter = 0;
#endif
    }
    /* init intr controller */
    rt_hw_pic_init();
}
