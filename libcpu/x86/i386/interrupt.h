/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-16     JasonHu      first version
 */

#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#define MAX_INTR_NR 0x81
#define EXCEPTION_INTR_BASE 0x00
#define IRQ_INTR_BASE 0x20
#define SYSCALL_INTR_BASE 0x80

#define MAX_IRQ_NR 16

#define EXCEPTION_PAGE_FAULT 14

#include "irq.h"
#include "i386.h"

#endif  /* __INTERRUPT_H__ */
