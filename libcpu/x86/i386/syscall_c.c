/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-03     lizhirui     first version
 * 2021-07-27     JasonHu      port to i386
 */

#include <rthw.h>
#include <rtthread.h>

//#define DBG_LEVEL DBG_WARNING
//#define DBG_LEVEL DBG_INFO
#include <rtdbg.h>

#ifdef RT_USING_USERSPACE

#include <stdint.h>
#include <mmu.h>
#include <page.h>
#include <lwp_mm_area.h>
#include <lwp_user_mm.h>
#include <lwp_arch.h>

#include "stackframe.h"

typedef rt_size_t (*syscallfunc_t)(rt_size_t,rt_size_t,rt_size_t,rt_size_t,rt_size_t,rt_size_t,rt_size_t);
syscallfunc_t lwp_get_sys_api(uint32_t);

void rt_hw_syscall_dispath(struct rt_hw_stack_frame *frame)
{
    if(frame->eax == 0)
    {
        rt_kprintf("syscall id = 0!\n");
        while(1);   // TODO: raise signal
    }

    if(frame->eax == 0xdeadbeef)
    {
        rt_kprintf("syscall id = 0xdeadbeef\n");
        while(1);   // TODO: raise signal
    }

#ifdef RT_USING_SIGNALS
    if(frame->eax == SIGNAL_RETURN_SYSCAL_ID) /* signal return */
    {
        lwp_signal_do_return(frame);
        return;
    }
#endif  /* RT_USING_SIGNALS */

    syscallfunc_t syscallfunc = (syscallfunc_t)lwp_get_sys_api(frame->eax);

    if(syscallfunc == RT_NULL)
    {
        rt_kprintf("unsupported syscall %d!\n", frame->eax);
        while(1);   // TODO: raise signal
    }
    /* TODO: support arg6 */
    LOG_I("\033[36msyscall id = %d,arg0 = 0x%p,arg1 = 0x%p,arg2 = 0x%p,arg3 = 0x%p,arg4 = 0x%p,arg5 = 0x%p,arg6 = 0x%p(unsupport)\n\033[37m",
        frame->eax, frame->ebx, frame->ecx, frame->edx, frame->esi, frame->edi, frame->ebp, 0);
    frame->eax = syscallfunc(frame->ebx, frame->ecx, frame->edx, frame->esi, frame->edi, frame->ebp, 0);
    LOG_I("\033[36msyscall deal ok,ret = 0x%p\n\033[37m",frame->eax);
}

#endif /* RT_USING_USERSPACE */
