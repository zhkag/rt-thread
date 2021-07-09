/*
* Copyright (c) 2006-2018, RT-Thread Development Team
*
* SPDX-License-Identifier: Apache-2.0
*/
#include "lwp_pty.h"

static struct rt_ptmx_device ptmx, *_ptmx = &ptmx;

rt_inline struct rt_wqueue *ptmx_get_wq(struct rt_lwp *lwp)
{
    if (lwp == RT_NULL)
    {
        return &_ptmx->wq;
    }
    return &lwp->wait_queue;
}

/* 查找空闲 pts 设备 */
static struct rt_pts_device *find_freepts(void)
{
    for(int i = 0; i < LWP_PTY_PTS_SIZE; i++)
    {
        if (_ptmx->pts[i].flags == PTY_INIT_FLAG_NONE)
        {
            _ptmx->pts[i].flags = PTY_INIT_FLAG_ALLOCED;
            return &_ptmx->pts[i];
        }
    }
    return RT_NULL;
}

/* 通过 fd(ptmx) 获取 pts 设备句柄 */
static struct rt_pts_device *ptmxfd2pts(struct dfs_fd *fd)
{
    int ptmx_fd;

    ptmx_fd = fd_get_fd_index(fd);
    for(int i = 0; i < LWP_PTY_PTS_SIZE; i++)
    {
        if (_ptmx->pts[i].flags != PTY_INIT_FLAG_NONE)
        {
            if (_ptmx->pts[i].ptmx_fd == ptmx_fd)
            {
                return &_ptmx->pts[i];
            }
        }
    }
    return RT_NULL;
}

static rt_size_t ptmx_read(struct rt_pts_device *pts, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_base_t level = 0;
    rt_size_t len = 0;

    level = rt_hw_interrupt_disable();
    if (size)
    {
        len = rt_ringbuffer_data_len(&pts->srb);
        if (len > size)
        {
            len = size;
        }
        if (len)
        {
            len = rt_ringbuffer_get(&pts->srb, buffer, len);
        }
    }
    rt_hw_interrupt_enable(level);

    return len;
}

static rt_size_t ptmx_write(struct rt_pts_device *pts, rt_off_t pos, const void *buffer, rt_size_t count)
{
    rt_base_t level = 0;
    rt_size_t size = 0;

    if (*(char *)buffer == '\r')
    {
        *(char *)buffer = '\n';
    }

    level = rt_hw_interrupt_disable();
    size = lwp_pts_push_mrb(pts, (void *)buffer, count);
    rt_hw_interrupt_enable(level);

    return size;
}

static int ptmx_file_open(struct dfs_fd *fd)
{
    rt_base_t level = 0;
    int ret = -1;
    struct rt_ptmx_device *ptmx = LWP_PTY_GET_PTMX(fd);
    struct rt_pts_device *pts = RT_NULL;
    int ptmx_fd = -1;
    struct rt_device *device = RT_NULL;
    struct rt_lwp *lwp = RT_NULL;
    struct rt_wqueue *wq = RT_NULL;

    level = rt_hw_interrupt_disable();

    pts = find_freepts();
    if (pts == RT_NULL)
    {
        LOG_E("no pts device, maximum number is %d", LWP_PTY_PTS_SIZE);
        ret = (-ENODEV);
        goto _exit;
    }

    /* 注册 pts 设备 */
    ptmx_fd = fd_get_fd_index(fd);
    if (ptmx_fd < 0)
    {
        pts->flags = PTY_INIT_FLAG_NONE; /* free pts */
    }
    ret = lwp_pts_register(pts, ptmx_fd, ptmx->pts_index);
    if (ret != RT_EOK)
    {
        fd_release(ptmx_fd);
        pts->flags = PTY_INIT_FLAG_NONE; /* free pts */
        LOG_E("register pts%d fail", ptmx->pts_index);
        ret = (-EIO);
        goto _exit;
    }

    /* 打开设备 */
    device = (struct rt_device *)fd->fnode->data;
    if (fd->fnode->ref_count == 1)
    {
        ret = rt_device_open(device, fd->flags);
        RT_ASSERT(ret == 0);
    }

    lwp = (struct rt_lwp *)(rt_thread_self()->lwp);
    wq = ptmx_get_wq(lwp);
    pts->swq = wq; /* 将当前等待队列设置到 pts 中，用于 ptmx 写数据后唤醒 lwp 读数据 */
    pts->ptmx_fd = ptmx_fd;
    ptmx->pts_index++;

    ret = 0;

_exit:
    rt_hw_interrupt_enable(level);
    return ret;
}

static int ptmx_file_close(struct dfs_fd *fd)
{
    rt_base_t level = 0;
    int ret = 0;
    struct rt_device *device = RT_NULL;
    struct rt_pts_device *pts = RT_NULL;

    level = rt_hw_interrupt_disable();
    if (fd->fnode->ref_count == 1)
    {
        pts = ptmxfd2pts(fd);
        if (pts && pts->mwq)
        {
            pts->ptmx_fd = RT_NULL;
            rt_wqueue_wakeup(pts->mwq, (void*)POLLIN);
        }
        device = (struct rt_device *)fd->fnode->data;
        ret = rt_device_close(device);
    }
    rt_hw_interrupt_enable(level);

    return ret;
}

static int ptmx_file_read(struct dfs_fd *fd, void *buf, size_t count)
{
    rt_base_t level = 0;
    size_t size = 0;
    struct rt_pts_device *pts = RT_NULL;
    int wait_ret = 0;

    level = rt_hw_interrupt_disable();

    pts = ptmxfd2pts(fd);
    if (pts)
    {
        while (count)
        {
            size = ptmx_read(pts, -1, buf, count);
            if (size > 0)
            {
                break;
            }

            if (fd->flags & O_NONBLOCK)
            {
                break;
            }

            /* 当直接读的时候，没有数据就挂起，等待 ptmx 写入数据后唤醒 */
            wait_ret = rt_wqueue_wait_interruptible(pts->swq, 0, RT_WAITING_FOREVER);
            if (wait_ret != 0)
            {
                break;
            }
        }

    }
    rt_hw_interrupt_enable(level);

    if (size < 0)
    {
        size = 0;
    }

    return size;
}

static int ptmx_file_write(struct dfs_fd *fd, const void *buf, size_t count)
{
    int size = 0;
    struct rt_pts_device *pts = RT_NULL;
    rt_base_t level = 0;

    level = rt_hw_interrupt_disable();

    pts = ptmxfd2pts(fd);
    if (pts)
    {
        size = ptmx_write(pts, -1, buf, count);
    }

    rt_hw_interrupt_enable(level);

    return size;
}

static int ptmx_file_ioctl(struct dfs_fd *fd, int cmd, void *args)
{
    rt_base_t level = 0;

    level = rt_hw_interrupt_disable();

    switch (cmd) {
    case TIOCSPTLCK:    /* Set PT Lock (disallow slave open) */
    {
        struct rt_pts_device *pts = ptmxfd2pts(fd);
        if (pts)
        {
            pts->pts_lock = *(int *)args;
            rt_hw_interrupt_enable(level);
            return 0;
        }
        else
        {
            rt_hw_interrupt_enable(level);
            return (-EIO);
        }
    }
    case TIOCGPTLCK:    /* Get PT Lock status */
    {
        struct rt_pts_device *pts = ptmxfd2pts(fd);
        if (pts)
        {
            *(int *)args = pts->pts_lock;
            rt_hw_interrupt_enable(level);
            return 0;
        }
        else
        {
            rt_hw_interrupt_enable(level);
            return (-EIO);
        }
    }
    case TIOCPKT:       /* Set PT packet mode */
        // return pty_set_pktmode(tty, (int __user *)arg);
        rt_hw_interrupt_enable(level);
        return 0;
    case TIOCGPKT:      /* Get PT packet mode */
        // return pty_get_pktmode(tty, (int __user *)arg);
        rt_hw_interrupt_enable(level);
        return 0;
    case TIOCGPTN:      /* Get PT Number */
    {
        struct rt_pts_device *pts = ptmxfd2pts(fd);
        if (pts)
        {
            /* 获取 ptmx 对应的 pts 的编号 */
            *(int *)args = pts->pts_index;
            rt_hw_interrupt_enable(level);
            return 0;
        }
        else
        {
            rt_hw_interrupt_enable(level);
            return (-EIO);
        }
    }
    case TIOCSIG:       /* Send signal to other side of pty */
        // return pty_signal(tty, (int) arg);
        rt_hw_interrupt_enable(level);
        return 0;

#if defined(RT_USING_POSIX_TERMIOS)
    case TCSETS:
    {
        // pts = ptmxfd2pts(fd);
        // rt_memcpy(&pts->tio, args, sizeof(struct termios));
        rt_hw_interrupt_enable(level);
        return (-EINVAL);
    }
    case TCGETS:
    {
        // pts = ptmxfd2pts(fd);
        // rt_memcpy(args, &pts->tio, sizeof(struct termios));
        rt_hw_interrupt_enable(level);
        return (-EINVAL);
    }
#endif /* RT_USING_POSIX_TERMIOS */
    }

    rt_hw_interrupt_enable(level);
    return (-EINVAL);
}

static int ptmx_file_poll(struct dfs_fd *fd, struct rt_pollreq *req)
{
    rt_base_t level = 0;
    int mask = POLLOUT;
    rt_size_t len;
    struct rt_pts_device *pts = RT_NULL;

    level = rt_hw_interrupt_disable();

    pts = ptmxfd2pts(fd);
    if (!pts)
    {
        mask |= POLLIN;
        goto _exit;
    }

    rt_poll_add(pts->swq, req);

    /* 判断是否有数据可以 read 设备 */
    len = rt_ringbuffer_data_len(&pts->srb);
    if (len)
    {
        mask |= POLLIN;
    }

_exit:
    rt_hw_interrupt_enable(level);

    return mask;
}

static rt_err_t ptmx_device_init(struct rt_device *dev)
{
    return RT_EOK;
}

static rt_err_t ptmx_device_open(struct rt_device *dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t ptmx_device_close(struct rt_device *dev)
{
    return RT_EOK;
}

static rt_size_t ptmx_device_read(struct rt_device *dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    return size;
}

static rt_size_t ptmx_device_write(struct rt_device *dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    return size;
}

static rt_err_t ptmx_device_control(rt_device_t dev, int cmd, void *args)
{
    return RT_EOK;
}

#ifdef RT_USING_POSIX
const static struct dfs_file_ops ptmx_file_ops =
{
    ptmx_file_open,
    ptmx_file_close,
    ptmx_file_ioctl,
    ptmx_file_read,
    ptmx_file_write,
    RT_NULL, /* flush */
    RT_NULL, /* lseek */
    RT_NULL, /* getdents */
    ptmx_file_poll,
};
#endif /* RT_USING_POSIX */

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops ptmx_device_ops =
{
    ptmx_device_init,
    ptmx_device_open,
    ptmx_device_close,
    ptmx_device_read,
    ptmx_device_write,
    ptmx_device_control,
};
#endif /* RT_USING_DEVICE_OPS */

static int lwp_ptmx_register(void)
{
    rt_err_t ret = RT_EOK;
    rt_base_t level = 0;
    struct rt_device *device = RT_NULL;

    level = rt_hw_interrupt_disable();

    if (_ptmx->flags != PTY_INIT_FLAG_NONE)
    {
        ret = (-RT_EBUSY);
        goto _exit;
    }

    device = &_ptmx->parent;
    device->type    = RT_Device_Class_Char;
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &ptmx_device_ops;
#else
    device->init    = ptmx_device_init;
    device->open    = ptmx_device_open;
    device->close   = ptmx_device_close;
    device->read    = ptmx_device_read;
    device->write   = ptmx_device_write;
    device->control = ptmx_device_control;
#endif /* RT_USING_DEVICE_OPS */

    ret = rt_device_register(device, "ptmx", RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK)
    {
        ret = -RT_EIO;
        goto _exit;
    }

#ifdef RT_USING_POSIX
    /* set fops */
    device->fops = &ptmx_file_ops;
#endif

    rt_wqueue_init(&_ptmx->wq);
    rt_mutex_init(&_ptmx->mutex, "ptmx", RT_IPC_FLAG_FIFO);
    rt_memset(_ptmx->pts, 0x00, sizeof(struct rt_pts_device)*LWP_PTY_PTS_SIZE);
    _ptmx->pts_index = 0;
    _ptmx->flags = PTY_INIT_FLAG_REGED;

_exit:
    rt_hw_interrupt_enable(level);

    return ret;
}
INIT_DEVICE_EXPORT(lwp_ptmx_register);
