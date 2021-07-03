/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-06-16     songchao   first version
 */

#ifndef __DRV_ETH_H__
#define __DRV_ETH_H__

#include <rtthread.h>
#include <netif/ethernetif.h>
#include "fsl_phy.h"
#include "imx6ull.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_ADDR_LEN 6
struct rt_imx6ul_ethps
{
    /* inherit from ethernet device */
    struct eth_device parent;
    /* interface address info, hw address */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];
    /* ETH_Speed */
    uint32_t    ETH_Speed;
    /* ETH_Duplex_Mode */
    uint32_t    ETH_Mode;
    rt_bool_t phy_link_status;
};


#ifdef __cplusplus
}
#endif

#endif /* __DRV_ETH_H__ */
