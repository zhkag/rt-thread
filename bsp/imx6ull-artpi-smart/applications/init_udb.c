/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022/01/20     bernard      the first version
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <rtthread.h>

#ifdef PKG_USING_UDBD
#include <udbd.h>

int udbd_system_init(void)
{
    udbd_init(UDBD_LINK_SOCKET, "e1");
    return 0;
}
INIT_APP_EXPORT(udbd_system_init);
#endif
