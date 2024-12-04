/*
 * Copyright (c) 2006-2024 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-04     zhkag        first version
 */

#include <rtthread.h>
#ifdef RT_USING_DFS_RAMFS
#include <dfs_fs.h>

extern struct dfs_ramfs *dfs_ramfs_create(rt_uint8_t *pool, rt_size_t size);

int mnt_init(void)
{
	rt_uint8_t *pool = RT_NULL;
	rt_size_t size = 1024;

	pool = rt_malloc(size);
	if (pool == RT_NULL)
		return 0;

	if (dfs_mount(RT_NULL, "/", "ram", 0, (const void *)dfs_ramfs_create(pool, size)) == 0)
		rt_kprintf("RAM file system initializated!\n");
	else
		rt_kprintf("RAM file system initializate failed!\n");

	return 0;
}
INIT_ENV_EXPORT(mnt_init);
#endif /* RT_USING_DFS_RAMFS */
