/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-17     JasonHu      first version
 */

#ifndef __X86_MMU_H__
#define __X86_MMU_H__

#include <rtdef.h>

#undef PAGE_SIZE

#define ADDRESS_WIDTH_BITS 32
#define PHYSICAL_ADDRESS_WIDTH_BITS ADDRESS_WIDTH_BITS
#define ARCH_ADDRESS_WIDTH_BITS ADDRESS_WIDTH_BITS

#define __SIZE(bit) (1U << (bit))
#define __MASK(bit) (__SIZE(bit) - 1UL)
#define __UMASK(bit) (~(__MASK(bit)))
#define __MASKVALUE(value,maskvalue) ((value) & (maskvalue))
#define __UMASKVALUE(value,maskvalue) ((value) & (~(maskvalue)))
#define __CHECKUPBOUND(value,bit_count) (!(((rt_size_t)(value)) & (~__MASK(bit_count))))
#define __CHECKALIGN(value,start_bit) (!(((rt_size_t)(value)) & (__MASK(start_bit))))

#define __PARTBIT(value,start_bit,length) (((value) >> (start_bit)) & __MASK(length))

#define __ALIGNUP(value,bit) (((value) + __MASK(bit)) & __UMASK(bit))
#define __ALIGNDOWN(value,bit) ((value) & __UMASK(bit))

#define PAGE_OFFSET_SHIFT 0
#define PAGE_OFFSET_BIT 12
#define PAGE_SIZE __SIZE(PAGE_OFFSET_BIT)
#define PAGE_OFFSET_MASK __MASK(PAGE_OFFSET_BIT)
#define PAGE_ADDR_MASK __UMASK(PAGE_OFFSET_BIT)

#define PTE_SHIFT (PAGE_OFFSET_SHIFT + PAGE_OFFSET_BIT)
#define PTE_BIT 10
#define PDE_SHIFT (PTE_SHIFT + PTE_BIT)
#define PDE_BIT 10

#define mmu_flush_tlb() \
    do \
    { \
        unsigned long tmpreg; \
        __asm__ __volatile__ ( \
                    "movl   %%cr3,  %0  \n\t" \
                    "movl   %0, %%cr3   \n\t" \
                    :"=r"(tmpreg) \
                    : \
                    :"memory" \
                    ); \
    } \
    while(0)

#define ARCH_PAGE_SIZE PAGE_SIZE
#define ARCH_PAGE_MASK (ARCH_PAGE_SIZE - 1)
#define ARCH_PAGE_SHIFT PAGE_OFFSET_BIT

void mmu_set_pagetable(rt_ubase_t addr);
void mmu_enable_user_page_access();
void mmu_disable_user_page_access();
void mmu_enable();

#endif  /* __X86_MMU_H__ */
