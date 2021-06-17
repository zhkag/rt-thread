/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-18     Jesven       first version
 * 2021-02-03     lizhirui     port to riscv64
 * 2021-02-06     lizhirui     add thread filter
 * 2021-02-19     lizhirui     port to new version of rt-smart
 * 2021-03-02     lizhirui     add a auxillary function for interrupt
 * 2021-03-04     lizhirui     delete thread filter
 * 2021-03-04     lizhirui     modify for new version of rt-smart
 */

#include <rthw.h>
#include <stddef.h>

#ifdef RT_USING_USERSPACE

#include <mmu.h>
#include <page.h>
#include <lwp_mm_area.h>
#include <lwp_user_mm.h>
#include <lwp_arch.h>

extern size_t MMUTable[];

//该函数用于支持用户栈扩展功能
int arch_expand_user_stack(void *addr)
{
    int ret = 0;
    size_t stack_addr = (size_t)addr;

    stack_addr &= ~PAGE_OFFSET_MASK;
    if ((stack_addr >= (size_t)USER_STACK_VSTART) && (stack_addr < (size_t)USER_STACK_VEND))
    {
        void *map = lwp_map_user(lwp_self(), (void *)stack_addr, PAGE_SIZE, RT_FALSE);

        if (map || lwp_user_accessable(addr, 1))
        {
            ret = 1;
        }
    }
    return ret;
}

void *lwp_copy_return_code_to_user_stack()
{
    void lwp_thread_return();
    void lwp_thread_return_end();
    rt_thread_t tid = rt_thread_self();

    if (tid->user_stack != RT_NULL)
    {
        size_t size = (size_t)lwp_thread_return_end - (size_t)lwp_thread_return;
        size_t userstack = (size_t)tid->user_stack + tid->user_stack_size - size;
        memcpy((void *)userstack, lwp_thread_return, size);
        return (void *)userstack;
    }

    return RT_NULL;
}

uint32_t lwp_fix_sp(uint32_t cursp)
{
    void lwp_thread_return();
    void lwp_thread_return_end();

    if (cursp == 0)
    {
        return 0;
    }

    return cursp - ((size_t)lwp_thread_return_end - (size_t)lwp_thread_return);
}

rt_thread_t rt_thread_sp_to_thread(void *spmember_addr)
{
    return (rt_thread_t)(((rt_ubase_t)spmember_addr) - (offsetof(struct rt_thread, sp)));
}

void *get_thread_kernel_stack_top(rt_thread_t thread)
{
    return (void *)(((rt_size_t)thread->stack_addr) + ((rt_size_t)thread->stack_size));
}

//don't support this temporarily in riscv
void *lwp_get_user_sp()
{
    return RT_NULL;
}

//该函数负责初始化lwp的页表和堆
int arch_user_space_init(struct rt_lwp *lwp)
{
    size_t *mmu_table;

    mmu_table = (size_t *)rt_pages_alloc(0);
    if (!mmu_table)
    {
        return -1;
    }

    lwp->end_heap = USER_HEAP_VADDR;
    memcpy(mmu_table, MMUTable, PAGE_SIZE);
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, mmu_table, 4 * PAGE_SIZE);
    rt_hw_mmu_map_init(&lwp->mmu_info, (void *)0x100000000UL, 0xFFFFFFFEFFFFFFFFUL, (rt_size_t *)mmu_table, 0);

    return 0;
}

//该函数负责返回内核页表指针
void *arch_kernel_mmu_table_get(void)
{
    return (void *)((char *)MMUTable);
}

void arch_user_space_vtable_free(struct rt_lwp *lwp)
{
    if (lwp && lwp->mmu_info.vtable)
    {
        rt_pages_free(lwp->mmu_info.vtable, 0);
    }
}

#endif
