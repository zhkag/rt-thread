/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020/10/7      bernard      the first version
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <rtdevice.h>
#include "drv_pin.h"

#define LED_PIN     GET_PIN(5, 3)

int main(void)
{
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);

    printf("hello rt-smart\n");

    for(;;)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
    return 0;
}

