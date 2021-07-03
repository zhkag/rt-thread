/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-06-16     songchao   support emac driver
 * 2021-06-29     songchao   add phy link detect
 */

#include "drv_eth.h"
#define DBG_TAG "drv.enet"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static struct rt_imx6ul_ethps imx6ul_eth_device;
static status_t read_data_from_eth(void *read_data,uint16_t *read_length);
static enet_config_t config;
static enet_handle_t g_handle;
static ENET_Type *enet_base_addr;
static uint8_t rx_recv_buf[ENET_RX_MAX_BUFFER_SIZE];

void imx6ul_eth_link_change(rt_bool_t up)
{
    if(up)
    {
        LOG_D("enet link up\n");
        eth_device_linkchange(&imx6ul_eth_device.parent, RT_TRUE);
        imx6ul_eth_device.phy_link_status = RT_TRUE;
    }
    else
    {
        LOG_D("enet link down\n");
        eth_device_linkchange(&imx6ul_eth_device.parent, RT_FALSE);
        imx6ul_eth_device.phy_link_status = RT_FALSE;
    }
}

enet_buffer_config_t buffConfig = {
    ENET_RXBD_NUM,
    ENET_TXBD_NUM,
    ENET_RXBUFF_ALIGN_SIZE,
    ENET_TXBUFF_ALIGN_SIZE,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    ENET_RXBUFF_TOTAL_SIZE,
    ENET_TXBUFF_TOTAL_SIZE,
};

void ENET_InitModuleClock(void)
{
    const clock_enet_pll_config_t config = {true, true, false, 1, 1};
    CLOCK_InitEnetPll(&config);
}

rt_err_t enet_buffer_init(enet_buffer_config_t *buffConfig)
{
    void *tx_buff_addr = RT_NULL;
    void *rx_buff_addr = RT_NULL;
    void *tx_bd_addr = RT_NULL;
    void *rx_bd_addr = RT_NULL;

    if(((SYS_PAGE_SIZE<<RX_BUFFER_INDEX_NUM)<buffConfig->rxBufferTotalSize)||
       ((SYS_PAGE_SIZE<<TX_BUFFER_INDEX_NUM)<buffConfig->txBufferTotalSize))
    {
        LOG_E("ERROR: alloc mem not enough for enet driver\n");
        return RT_ERROR;
    }
    rx_buff_addr = rt_pages_alloc(RX_BUFFER_INDEX_NUM);
    if(!rx_buff_addr)
    {
        LOG_E("ERROR: rx buff page alloc failed\n");
        return RT_ERROR;
    }
    buffConfig->rxBufferAlign = (void *)rt_ioremap_nocache(virtual_to_physical(rx_buff_addr), (SYS_PAGE_SIZE<<RX_BUFFER_INDEX_NUM));
    buffConfig->rxPhyBufferAlign = (void *)virtual_to_physical(rx_buff_addr);

    tx_buff_addr = rt_pages_alloc(TX_BUFFER_INDEX_NUM);
    if(!tx_buff_addr)
    {
        LOG_E("ERROR: tx buff page alloc failed\n");
        return RT_ERROR;
    }
    buffConfig->txBufferAlign = (void *)rt_ioremap_nocache(virtual_to_physical(tx_buff_addr), (SYS_PAGE_SIZE<<TX_BUFFER_INDEX_NUM));
    buffConfig->txPhyBufferAlign = (void *)virtual_to_physical(tx_buff_addr);


    rx_bd_addr = rt_pages_alloc(RX_BD_INDEX_NUM);
    if(!rx_bd_addr)
    {
        LOG_E("ERROR: rx bd page alloc failed\n");
        return RT_ERROR;
    }
    buffConfig->rxBdStartAddrAlign = (void *)rt_ioremap_nocache(virtual_to_physical(rx_bd_addr), (SYS_PAGE_SIZE<<RX_BD_INDEX_NUM));
    buffConfig->rxPhyBdStartAddrAlign = virtual_to_physical(rx_bd_addr);

    tx_bd_addr = rt_pages_alloc(TX_BD_INDEX_NUM);
    if(!tx_bd_addr)
    {
        LOG_E("ERROR: tx bd page alloc failed\n");
        return RT_ERROR;
    }
    buffConfig->txBdStartAddrAlign = (void *)rt_ioremap_nocache(virtual_to_physical(tx_bd_addr), (SYS_PAGE_SIZE<<TX_BD_INDEX_NUM));
    buffConfig->txPhyBdStartAddrAlign = virtual_to_physical(tx_bd_addr);

    return RT_EOK;
}

/* EMAC initialization function */
static rt_err_t rt_imx6ul_eth_init(rt_device_t dev)
{
    bool link = false;
    rt_err_t state;
    phy_speed_t speed;
    phy_duplex_t duplex;

    enet_base_addr = (ENET_Type *)rt_ioremap((void *)IMX6UL_ENET,SYS_PAGE_SIZE);

    phy_reset();
    ENET_InitPins();
    ENET_InitModuleClock();
    ENET_GetDefaultConfig(&config);
    config.interrupt |= (ENET_RX_INTERRUPT);
    PHY_Init(enet_base_addr, ENET_PHY, SYS_CLOCK_HZ);
    PHY_GetLinkStatus(enet_base_addr, ENET_PHY, &link);
    if (link)
    {
        /* Get the actual PHY link speed. */
        PHY_GetLinkSpeedDuplex(enet_base_addr, ENET_PHY, &speed, &duplex);
        /* Change the MII speed and duplex for actual link status. */
        config.miiSpeed = (enet_mii_speed_t)speed;
        config.miiDuplex = (enet_mii_duplex_t)duplex;
    }
    else
    {
        LOG_E("\r\nPHY Link down, please check the cable connection and link partner setting.\r\n");
    }

    state = enet_buffer_init(&buffConfig);
    if(state != RT_EOK)
    {
        return state;
    }
    ENET_Init(enet_base_addr, &g_handle, &config, &buffConfig, &imx6ul_eth_device.dev_addr[0], SYS_CLOCK_HZ);
    ENET_ActiveRead(enet_base_addr);
    rt_hw_interrupt_install(IMX_INT_ENET, (rt_isr_handler_t)ENET_DriverIRQHandler, (void *)enet_base_addr,ENET_IRQ_NAME);
    rt_hw_interrupt_umask(IMX_INT_ENET);

    return RT_EOK;
}

static rt_err_t rt_imx6ul_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_imx6ul_eth_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t rt_imx6ul_eth_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    return 0;
}

static rt_size_t rt_imx6ul_eth_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    return 0;
}

static rt_err_t rt_imx6ul_eth_control(rt_device_t dev, int cmd, void *args)
{
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get MAC address */
        if (args)
        {
            rt_memcpy(args, imx6ul_eth_device.dev_addr, MAX_ADDR_LEN);
        }
        else
        {
            return -RT_ERROR;
        }
        break;

    default :
        break;
    }
    return RT_EOK;
}

/* transmit data*/
rt_err_t rt_imx6ul_eth_tx(rt_device_t dev, struct pbuf *p)
{
    rt_err_t ret = RT_ERROR;
    struct pbuf *q = NULL;
    uint16_t offset = 0;
    uint32_t last_flag = 0;
    status_t status;
    RT_ASSERT(p);

    for(q = p;q != NULL;q=q->next)
    {
        if(q->next == NULL)
        {
            last_flag = 1;
        }
        else
        {
            last_flag = 0;
        }
        status = ENET_SendFrame(enet_base_addr, &g_handle, q->payload, q->len,last_flag);
        offset = offset + q->len;
        if(status == kStatus_Success)
        {
        }
        else
        {
            return RT_ERROR;
        }
    }
    if(offset > ENET_FRAME_MAX_FRAMELEN)
    {
        LOG_E("net error send length %d exceed max length\n",offset);
    }
    return ret;
}

struct pbuf *rt_imx6ul_eth_rx(rt_device_t dev)
{
    struct pbuf *p = NULL;
    status_t status;
    uint16_t length  =0;
    status = read_data_from_eth(rx_recv_buf,&length);
    if(status == kStatus_ENET_RxFrameEmpty)
    {
        return RT_NULL;
    }
    else if(status == kStatus_ENET_RxFrameError)
    {
        return RT_NULL;
    }
    if(length > ENET_FRAME_MAX_FRAMELEN)
    {
        LOG_E("net error recv length %d exceed max length\n",length);
        return RT_NULL;
    }

    p = pbuf_alloc(PBUF_RAW, ENET_FRAME_MAX_FRAMELEN, PBUF_POOL);
    if(p == RT_NULL)
    {
        return RT_NULL;
    }
    else
    {
        pbuf_realloc(p, length);
        memcpy(p->payload,rx_recv_buf,length);
        p->len = length;
        p->tot_len = length;
    }
    return p;
}

void rx_enet_callback()
{
    eth_device_ready(&(imx6ul_eth_device.parent));
    ENET_DisableInterrupts(enet_base_addr,ENET_RX_INTERRUPT);
}

void tx_enet_callback()
{
    ENET_DisableInterrupts(enet_base_addr,ENET_TX_INTERRUPT);
}

static status_t read_data_from_eth(void *read_data,uint16_t *read_length)
{
    status_t status = 0;
    uint16_t length = 0;
    /* Get the Frame size */
    status = ENET_ReadFrame(enet_base_addr,&g_handle,&config,read_data,&length);
    if((status == kStatus_ENET_RxFrameEmpty)||(status == kStatus_ENET_RxFrameError))
    {
        ENET_EnableInterrupts(enet_base_addr,ENET_RX_INTERRUPT);
        if(status == kStatus_ENET_RxFrameError)
        {
            /*recv error happend reinitialize mac*/
            ENET_Init(enet_base_addr, &g_handle, &config, &buffConfig, &imx6ul_eth_device.dev_addr[0], SYS_CLOCK_HZ);
            ENET_ActiveRead(enet_base_addr);
            return kStatus_ENET_RxFrameError;
        }
        else if(status == kStatus_ENET_RxFrameEmpty)
        {
            return kStatus_ENET_RxFrameEmpty;
        }
    }
    *read_length = length;
    return status;
}
/*phy link detect thread*/
static void phy_detect_thread_entry(void *param)
{
    bool link = false;
    while(1)
    {
        PHY_GetLinkStatus(enet_base_addr, ENET_PHY, &link);
        if(link != imx6ul_eth_device.phy_link_status)
        {
            if(link == true)
            {
                PHY_StartNegotiation(enet_base_addr,ENET_PHY);
            }
            imx6ul_eth_link_change(link);
        }
        rt_thread_delay(DETECT_DELAY_ONE_SECOND);
    }
}

_internal_ro struct rt_device_ops _k_enet_ops =
{
    rt_imx6ul_eth_init,
    rt_imx6ul_eth_open,
    rt_imx6ul_eth_close,
    rt_imx6ul_eth_read,
    rt_imx6ul_eth_write,
    rt_imx6ul_eth_control,
};

static int imx6ul_eth_init(void)
{
    rt_err_t state = RT_EOK;

    imx6ul_eth_device.dev_addr[0] = 0xa8;
    imx6ul_eth_device.dev_addr[1] = 0x5e;
    imx6ul_eth_device.dev_addr[2] = 0x45;
    imx6ul_eth_device.dev_addr[3] = 0x31;
    imx6ul_eth_device.dev_addr[4] = 0x32;
    imx6ul_eth_device.dev_addr[5] = 0x33;

    imx6ul_eth_device.parent.parent.ops = &_k_enet_ops;

    imx6ul_eth_device.parent.eth_rx     = rt_imx6ul_eth_rx;
    imx6ul_eth_device.parent.eth_tx     = rt_imx6ul_eth_tx;
    imx6ul_eth_device.phy_link_status   = RT_FALSE;

    /* register eth device */
    state = eth_device_init(&(imx6ul_eth_device.parent), ENET_NAME);
    if (RT_EOK == state)
    {
        LOG_E("emac device init success\n");
    }
    else
    {
        LOG_E("emac device init faild: %d", state);
        state = -RT_ERROR;
    }

    /* start phy link detect */
    rt_thread_t phy_link_tid;
    phy_link_tid = rt_thread_create("link_detect",
                            phy_detect_thread_entry,
                            RT_NULL,
                            512,
                            RT_THREAD_PRIORITY_MAX - 2,
                            2);
    if (phy_link_tid != RT_NULL)
    {
        rt_thread_startup(phy_link_tid);
    }
    return state;
}
INIT_DEVICE_EXPORT(imx6ul_eth_init);
