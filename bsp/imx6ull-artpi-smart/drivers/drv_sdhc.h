/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-23     Lyons        first version
 */

#ifndef __DRV_SDHC_H__
#define __DRV_SDHC_H__

#include "board.h"
#include "drv_common.h"

#ifdef BSP_USING_SDHC

/* Fixed value, not edit! */
#define SDCARD_CONTROL_PIN_NUM      (6)

struct imx_periph
{
    rt_uint32_t                 paddr;
    rt_uint32_t                 vaddr;
};

struct imx_sddev
{
    struct rt_device                parent;

    const char                      *name;
    struct imx_periph               periph;
    rt_uint32_t                     irqno;

    struct imx6ull_iomuxc           gpio[SDCARD_CONTROL_PIN_NUM];

    rt_uint32_t                     flag;
};

#endif //#ifdef BSP_USING_SDHC
#endif //#ifndef __DRV_SDHC_H__
