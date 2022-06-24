/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <fdt.h>
#if defined(RT_USING_POSIX)
#include <rtdevice.h> /* for wqueue_init */
#endif

/**
 * This function registers a device driver with specified name.
 *
 * @param drv the pointer of driver structure
 * @param device_id the id of the device
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_driver_device_match_with_id(const rt_driver_t drv,int device_id)
{
    rt_device_t device;
    int ret;
    if (!drv)
    {
        return -RT_EINVAL;
    }
    device = rt_device_create_since_driver(drv,device_id);
    if(!device)
    {
        return -RT_ERROR;
    }
    ret = rt_device_driver_bind(device,drv,RT_NULL);
    if(ret != 0)
    {
        return -RT_ERROR;
    }
    ret = rt_device_probe_and_init(device);
    if(ret != 0)
    {
        return -RT_ERROR;
    }
    return ret;
}

RTM_EXPORT(rt_driver_device_match_with_id);

#ifdef PKG_USING_FDT
/**
 * This function registers a device driver with specified name.
 *
 * @param drv the pointer of driver structure
 * @param from_node eth entry ftd node 
 * @param max_dev_num the max device support 
 * 
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_driver_device_match_with_dtb(const rt_driver_t drv,void *from_node,int max_dev_num)
{
    struct dtb_node** node_list; 
    rt_device_t device;
    int ret,i;
    int active_dev_num = 0;
    if ((!drv)||(!drv->dev_match)||(!drv->dev_match->compatible)||(!from_node))
    {
        return -RT_EINVAL;
    }

    node_list = rt_calloc(max_dev_num,sizeof(void *));
    if(!node_list)
    {
        return -RT_ERROR;
    }
    
    ret = fdt_find_all_active_compatible_node(from_node,drv->dev_match->compatible,node_list,max_dev_num,&active_dev_num);
    if((ret != 0) || (!active_dev_num))
    {
        return -RT_ERROR;
    }
    
    for(i = 0; i < active_dev_num; i ++)
    {
        device = rt_device_create_since_driver(drv,i);
        if(!device)
        {
            return -RT_ERROR;
        }
    
        ret = rt_device_driver_bind(device,drv,node_list[i]);
        if(ret != 0)
        {
            return -RT_ERROR;
        }
        ret = rt_device_probe_and_init(device);
        if(ret != 0)
        {
            return -RT_ERROR;
        }

    }

    return ret;
}

RTM_EXPORT(rt_driver_device_match_with_dtb);
#endif  

/**
 * This function removes a previously registered device driver
 *
 * @param dev the pointer of device driver structure
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_driver_unregister(const rt_driver_t drv)
{
    /*todo*/
    return RT_EOK;
}
RTM_EXPORT(rt_driver_unregister);

