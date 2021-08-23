/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021/08/19     bernard      the first version
 */

#include <rtthread.h>

#ifdef RT_USING_DFS
#include <dfs_fs.h>
#include <dfs_romfs.h>

int mnt_init(void)
{
    if (dfs_mount(RT_NULL, "/", "rom", 0, &romfs_root) != 0)
    {
        rt_kprintf("Dir / mount failed!\n");
        return -1;
    }

    rt_kprintf("file system initialization done!\n");
    return 0;
}
INIT_ENV_EXPORT(mnt_init);
#endif
