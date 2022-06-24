#ifndef __RT_DRIVER_H__
#define __RT_DRIVER_H__

#include <rtdef.h>

#define RT_DRIVER_MATCH_DTS (1<<0)

struct rt_device_id 
{
	const char *compatible;
	void *data;
};

struct rt_driver
{
#ifdef RT_USING_DEVICE_OPS    
    const struct rt_device_ops *dev_ops;
#endif    
    const struct filesystem_ops *fops;
    const char *name;
    enum rt_device_class_type dev_type;
    int device_priv_data_size;
    int device_size;
    int flag;
    const struct rt_device_id *dev_match;
	int (*probe)(struct rt_device *dev);
    int (*init)(struct rt_device *dev);
    int (*remove)(struct rt_device *dev);
	const void *ops;	/* driver-specific operations */
};
typedef struct rt_driver *rt_driver_t;

#endif