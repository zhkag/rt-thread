/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <fdt.h>
#include <rt_dtb_node.h>
#include <rt_device_node.h>
#if defined(RT_USING_POSIX)
#include <rtdevice.h> /* for wqueue_init */
#endif

#define MAX_COMPATIBLE_NUM 10

/**
 * This function  bind drvier and device
 *
 * @param driver the pointer of driver structure
 * @param device the pointer of device structure
 * @param node the pointer of fdt node structure
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_device_bind(rt_driver_t driver, rt_device_t device, void *node)
{
    if((!driver) || (!device))
    {
        return -RT_ERROR;
    }

    device->drv = driver;
    device->ops = driver->dev_ops;
    device->fdt_node = node;

    return RT_EOK;
} 

/**
 * This function  create rt_device and init the device
 *
 * @param driver the pointer of driver structure
 * @param device_num how many device should be create
 * @param ftd_node_list ftd node list
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_device_create_and_init(rt_driver_t drv,int device_num,struct dtb_node **ftd_node_list)
{
    int i = 0;
    int ret = -1;
    rt_device_t device;
    for(i = 0; i < device_num; i ++)
    {
        device = (rt_device_t)rt_malloc(drv->device_size);
        if(device == RT_NULL)
        {
            return -RT_ERROR;
        }
        rt_memset(device,0,drv->device_size);
        if(drv->device_priv_data_size != 0)
        {
            device->user_data = (void *)(rt_malloc(drv->device_priv_data_size));
            if(device->user_data == RT_NULL)
            {
                rt_free(device);
                return -RT_ERROR;
            }   
            rt_memset(device->user_data,0,drv->device_priv_data_size);
        }
        
        device->device_id = i;
        rt_strncpy(device->parent.name,drv->name,rt_strlen(drv->name));
        rt_sprintf(device->parent.name + rt_strlen(drv->name),"%d",i);

        rt_device_bind(drv,device,ftd_node_list[i]);

        if(device->drv->probe)
        {
            ret = device->drv->probe((rt_device_t)device);
        }
        if(device->drv->init)
        {
            ret = device->drv->init((rt_device_t)device);
        }

    }
    return ret;
}
/**
 * This function registers a device driver with specified name.
 *
 * @param dev the pointer of driver structure
 * @param flags the capabilities flag of device
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_driver_register(rt_driver_t drv)
{
    struct dtb_node *root_node = RT_NULL;
    struct dtb_node* node_list[MAX_COMPATIBLE_NUM]; 
    int ret,i,device_num;
    if (drv == RT_NULL)
    {
        return -RT_ERROR;
    }

    root_node = get_dtb_node_head();
    if(drv->dev_match->compatible != RT_NULL)
    {
        ret = fdt_find_all_active_compatible_node(root_node,drv->dev_match->compatible,node_list,MAX_COMPATIBLE_NUM,&device_num);
        if(!ret)
        {
        }
	}
    else
    {
        device_num = drv->total_device_num;  
        for(i = 0; i < device_num; i++)
        {
            node_list[i] = RT_NULL;
        }
    }

    if(!device_num)
    {
        rt_kprintf("can not match compatible device\n");
    }
    ret = rt_device_create_and_init(drv,device_num,node_list);

    return ret;
}

RTM_EXPORT(rt_driver_register);

/**
 * This function removes a previously registered device driver
 *
 * @param dev the pointer of device driver structure
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_driver_unregister(rt_driver_t drv)
{
    /*todo*/
    return RT_EOK;
}
RTM_EXPORT(rt_driver_unregister);

