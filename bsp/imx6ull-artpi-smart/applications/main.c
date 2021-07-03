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

#ifdef BSP_USING_LCD
#include "drv_lcd.h"

struct lcd_info info;

int imx6ull_elcd_test()
{
    struct rt_device *lcd_dev = RT_NULL;
    int buff_size = 0;
    rt_uint8_t *red_buff, *green_buff, *blue_buff;

    lcd_dev = (struct rt_device *)rt_device_find("lcd");

    RT_ASSERT(lcd_dev);

    rt_device_init(lcd_dev);
    rt_device_control(lcd_dev, RTGRAPHIC_CTRL_GET_INFO, &info);

    buff_size = info.graphic.width * info.graphic.height * info.graphic.bits_per_pixel /8;
    red_buff = (rt_uint8_t *)rt_malloc(buff_size);
    green_buff = (rt_uint8_t *)rt_malloc(buff_size);
    blue_buff = (rt_uint8_t *)rt_malloc(buff_size);

    for(int i = 0; i < buff_size / 2; i++)
    {
        red_buff[2 * i] = 0x00;
        red_buff[2 * i + 1] = 0x7c;

        green_buff[2 * i] = 0xE0;
        green_buff[2 * i + 1] = 0x07;

        blue_buff[2 * i] = 0x1F;
        blue_buff[2 * i + 1] = 0x00;
    }

    for(int i = 0; i < 50; i++)
    {
        rt_memcpy(info.graphic.framebuffer, red_buff, buff_size);
        rt_device_control(lcd_dev, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
        rt_thread_delay(50);

        rt_memcpy(info.graphic.framebuffer, green_buff, buff_size);
        rt_device_control(lcd_dev, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
        rt_thread_delay(50);

        rt_memcpy(info.graphic.framebuffer, blue_buff, buff_size);
        rt_device_control(lcd_dev, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
        rt_thread_delay(50);
    }

    rt_device_close(lcd_dev);

    return RT_EOK;
}
MSH_CMD_EXPORT(imx6ull_elcd_test, imx6ull_elcd_test);

#endif
