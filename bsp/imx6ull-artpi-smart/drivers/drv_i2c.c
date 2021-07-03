/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-13     Lyons        first version
 * 2021-06-23     RiceChen     refactor
 */

#include <rthw.h>
#include <rtdevice.h>

#ifdef BSP_USING_I2C

#define LOG_TAG              "drv.i2c"
#include <drv_log.h>

#if !defined(BSP_USING_I2C1) && !defined(BSP_USING_I2C2) && !defined(BSP_USING_I2C3) && !defined(BSP_USING_I2C4)
#error "Please define at least one BSP_USING_I2Cx"
#endif

#include "fsl_iomuxc.h"
#include "drv_i2c.h"

static struct imx6ull_i2c_config i2c_config[] =
{
#ifdef BSP_USING_I2C1
    I2C1_BUS_CONFIG,
#endif
#ifdef BSP_USING_I2C2
    I2C2_BUS_CONFIG,
#endif
#ifdef BSP_USING_I2C3
    I2C3_BUS_CONFIG,
#endif
#ifdef BSP_USING_I2C4
    I2C4_BUS_CONFIG,
#endif
};

static struct imx6ull_i2c_bus i2c_obj[sizeof(i2c_config) / sizeof(i2c_config[0])];

static rt_size_t imx6ull_i2c_mst_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct imx6ull_i2c_bus *i2c_bus = RT_NULL;
    i2c_master_transfer_t xfer = {0};
    rt_size_t i = 0;

    RT_ASSERT(bus != RT_NULL);
    
    i2c_bus = (struct imx6ull_i2c_bus *)bus;

    for(i = 0 ;i < num; i++)
    {
        if(msgs[i].flags & RT_I2C_RD)
        {
            xfer.flags = kI2C_TransferNoStartFlag;
            xfer.slaveAddress = msgs[i].addr;
            xfer.direction = kI2C_Read;
            xfer.subaddress = 0;
            xfer.subaddressSize = 0;
            xfer.data = msgs[i].buf;
            xfer.dataSize = msgs[i].len;
            I2C_MasterTransferBlocking(i2c_bus->config->I2C, &xfer);
        }
        else
        {
            xfer.flags = kI2C_TransferNoStartFlag;
            xfer.slaveAddress = msgs[i].addr;
            xfer.direction = kI2C_Write;
            xfer.subaddress = 0;
            xfer.subaddressSize = 0;
            xfer.data = msgs[i].buf;
            xfer.dataSize = msgs[i].len;
            I2C_MasterTransferBlocking(i2c_bus->config->I2C, &xfer);
        }
    }

    return i;
}

static rt_err_t imx6ull_i2c_bus_control(struct rt_i2c_bus_device *bus, rt_uint32_t cmd, rt_uint32_t arg)
{
    return RT_EOK;
}

static rt_err_t imx6ull_i2c_gpio_init(struct imx6ull_i2c_bus *bus)
{
    struct imx6ull_i2c_bus *i2c_bus = RT_NULL;

    i2c_bus = (struct imx6ull_i2c_bus *)bus;

    imx6ull_gpio_init(&i2c_bus->config->scl_gpio);
    imx6ull_gpio_init(&i2c_bus->config->sda_gpio);
    return RT_EOK;

}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_i2c_bus_device_ops imx6ull_i2c_ops = 
{
    .master_xfer = imx6ull_i2c_mst_xfer,
    .slave_xfer = RT_NULL,
    .i2c_bus_control = imx6ull_i2c_bus_control,
};
#endif

int rt_hw_i2c_init(void)
{
    rt_uint16_t obj_num = 0;
    rt_uint32_t src_clock;
    i2c_master_config_t masterConfig = {0};

    obj_num = sizeof(i2c_config) / sizeof(i2c_config[0]);

    src_clock = (CLOCK_GetFreq(kCLOCK_IpgClk) / (CLOCK_GetDiv(kCLOCK_PerclkDiv) + 1U));

    for(int i = 0; i < obj_num; i++)
    {
        i2c_obj[i].config = &i2c_config[i];
        i2c_obj[i].config->I2C = (I2C_Type *)imx6ull_get_periph_vaddr((rt_uint32_t)(i2c_obj[i].config->I2C));
        i2c_obj[i].parent.ops = &imx6ull_i2c_ops;
        imx6ull_i2c_gpio_init(&i2c_obj[i]);

        I2C_MasterGetDefaultConfig(&masterConfig);
        masterConfig.baudRate_Bps = i2c_config[i].baud_rate;

        CLOCK_EnableClock(i2c_obj[i].config->clk_ip_name);

        I2C_MasterInit(i2c_obj[i].config->I2C, &masterConfig, src_clock);

        rt_i2c_bus_device_register(&i2c_obj[i].parent, i2c_obj[i].config->name);
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_i2c_init);

#endif