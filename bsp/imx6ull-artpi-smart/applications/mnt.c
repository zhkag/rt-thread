/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rtthread.h>

#ifdef RT_USING_DFS

#include <dfs_fs.h>
#include <dfs_romfs.h>

int mnt_init(void)
{
#ifdef RT_USING_SDIO2
    int part_id = 1;
    rt_thread_mdelay(500);

    if (dfs_mount("emmc","/","ext",0,(void *)part_id) != 0)
    {
        rt_kprintf("Dir / mount failed!\n");
        return -1;
    }
    else
    {
        rt_kprintf("file system initialization done!\n");
    }
#else
    rt_thread_mdelay(500);
    if (dfs_mount(NULL, "/", "rom", 0, &romfs_root) != 0)
    {
        rt_kprintf("Dir / mount failed!\n");
        return -1;
    }

    sd_task();
    rt_kprintf("file system initialization done!\n");
#endif
    return 0;
}
INIT_APP_EXPORT(mnt_init);


#endif
