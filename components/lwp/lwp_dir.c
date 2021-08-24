/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-08-24     linzhenxing  the first version
 */
#include <dfs.h>
#include "lwp_dir.h"
#ifdef RT_USING_LWP
#include <lwp.h>
#endif

#ifdef DFS_USING_WORKDIR
extern char working_directory[];
#endif

void lwp_dir_set(char *buf)
{
    if(strlen(buf) >= DFS_PATH_MAX)
    {
        rt_kprintf("buf too long!\n");
        return ;
    }

#ifdef RT_USING_LWP
    struct rt_lwp *lwp;

    lwp = (struct rt_lwp *)rt_thread_self()->lwp;
    if (lwp)
        rt_strncpy(lwp->working_directory, buf, DFS_PATH_MAX);
    else
        rt_strncpy(working_directory, buf, DFS_PATH_MAX);
#else
    rt_strncpy(working_directory, buf, DFS_PATH_MAX);
#endif
    return ;
}

char *lwp_dir_get(void)
{
    char *dir_buf = RT_NULL;
#ifdef RT_USING_LWP
    struct rt_lwp *lwp;

    lwp = (struct rt_lwp *)rt_thread_self()->lwp;
    if (lwp)
        dir_buf = &lwp->working_directory[0];
    else
        dir_buf = &working_directory[0];
#else
    dir_buf =  &working_directory[0];
#endif
    return dir_buf;
}
