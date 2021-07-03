/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-11     Lyons        first version
 * 2021-06-24     RiceChen     refactor
 */

#include <rthw.h>
#include <rtdevice.h>

#ifdef BSP_USING_LCD

#define LOG_TAG              "drv.lcd"
#include <drv_log.h>

#include "fsl_iomuxc.h"
#include "drv_lcd.h"
#include <lwp_user_mm.h>

struct imx6ull_lcd_config lcd_config = LCD_BUS_CONFIG;
struct imx6ull_lcd_bus lcd_obj;

static rt_err_t imx6ull_elcd_init(rt_device_t device)
{
    struct imx6ull_lcd_bus *elcd_dev = RT_NULL;
    clock_video_pll_config_t pll_config;
    elcdif_rgb_mode_config_t lcd_config;

    RT_ASSERT(device != RT_NULL);

    elcd_dev = (struct imx6ull_lcd_bus *)device;

    pll_config.loopDivider = 32;
    pll_config.postDivider = 1;
    pll_config.numerator   = 0;
    pll_config.denominator = 0;

    CLOCK_InitVideoPll(&pll_config);

    lcd_config.hfp           = LCD_HFP;
    lcd_config.vfp           = LCD_VFP;
    lcd_config.hbp           = LCD_HBP;
    lcd_config.vbp           = LCD_VBP;
    lcd_config.hsw           = LCD_HSW;
    lcd_config.vsw           = LCD_VSW;

    lcd_config.polarityFlags = kELCDIF_DataEnableActiveHigh  |
                               kELCDIF_VsyncActiveLow      |
                               kELCDIF_HsyncActiveLow      |
                               kELCDIF_DriveDataOnRisingClkEdge;

    lcd_config.panelWidth    = LCD_WIDTH;
    lcd_config.panelHeight   = LCD_HEIGHT;
    lcd_config.pixelFormat   = kELCDIF_PixelFormatRGB565;
    lcd_config.dataBus       = kELCDIF_DataBus24Bit;
    lcd_config.bufferAddr    = (uint32_t)elcd_dev->fb_phy;

    ELCDIF_RgbModeInit(elcd_dev->config->ELCDIF, &lcd_config);
    ELCDIF_RgbModeStart(elcd_dev->config->ELCDIF);

    return RT_EOK;
}

static rt_err_t imx6ull_elcd_control(rt_device_t device, int cmd, void *args)
{
    struct imx6ull_lcd_bus *elcd_dev = RT_NULL;

    RT_ASSERT(device != RT_NULL);

    elcd_dev = (struct imx6ull_lcd_bus *)device;

    switch(cmd)
    {
        case RTGRAPHIC_CTRL_RECT_UPDATE:
        {
            break;
        }
        case RTGRAPHIC_CTRL_POWERON:
        {
            rt_pin_write(IMX6ULL_LCD_BL_PIN, PIN_HIGH);
            break;
        }
        case RTGRAPHIC_CTRL_POWEROFF:
        {
            rt_pin_write(IMX6ULL_LCD_BL_PIN, PIN_LOW);
            break;
        }
        case RTGRAPHIC_CTRL_GET_INFO:
        {
            struct lcd_info *info = (struct lcd_info *)args;
            RT_ASSERT(info != RT_NULL);

            rt_memcpy(&info->graphic, &elcd_dev->info, sizeof(struct rt_device_graphic_info));
            
            info->screen.shamem_len   = elcd_dev->info.width * elcd_dev->info.width * elcd_dev->info.bits_per_pixel/8;
            info->screen.shamem_start = (rt_uint32_t)lwp_map_user_phy(lwp_self(), RT_NULL, 
                                                                        elcd_dev->fb_phy,
                                                                        info->screen.shamem_len, 1);
            break;
        }
        case RTGRAPHIC_CTRL_SET_MODE:
        {
            break;
        }
    }
    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops elcd_ops =
{
    imx6ull_elcd_init,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    imx6ull_elcd_control,
};
#endif

int rt_hw_elcd_init(void)
{
    rt_err_t ret = 0;

    lcd_config.ELCDIF       = (LCDIF_Type *)imx6ull_get_periph_vaddr((rt_uint32_t)(lcd_config.ELCDIF));
    lcd_config.lcd_mux_base = (rt_uint32_t)imx6ull_get_periph_vaddr((rt_uint32_t)(lcd_config.lcd_mux_base));
    lcd_config.lcd_cfg_base = (rt_uint32_t)imx6ull_get_periph_vaddr((rt_uint32_t)(lcd_config.lcd_cfg_base));

    for(int i = 0; i < LCD_GPIO_MAX; i++)
    {
        IOMUXC_SetPinMux((lcd_config.lcd_mux_base + i * 4), 0x0U, 0x00000000U, 0x0U, (lcd_config.lcd_cfg_base + i * 4), 0);
	    IOMUXC_SetPinConfig((lcd_config.lcd_mux_base + i * 4), 0x0U, 0x00000000U, 0x0U, (lcd_config.lcd_cfg_base + i * 4), 0xB9);
    }

    CLOCK_EnableClock(lcd_config.apd_clk_name);
    CLOCK_EnableClock(lcd_config.pix_clk_name);

    lcd_obj.config = &lcd_config;

    lcd_obj.fb_virt = rt_pages_alloc(rt_page_bits(LCD_BUF_SIZE));
    lcd_obj.fb_phy = lcd_obj.fb_virt + PV_OFFSET;

    rt_kprintf("fb address => 0x%08x\n", lcd_obj.fb_phy);
    if(lcd_obj.fb_phy == RT_NULL)
    {
        rt_kprintf("initialize frame buffer failed!\n");
        return -RT_ERROR;
    }

    lcd_obj.info.width          = LCD_WIDTH;
    lcd_obj.info.height         = LCD_HEIGHT;
    lcd_obj.info.pixel_format   = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    lcd_obj.info.bits_per_pixel = LCD_BITS_PER_PIXEL;
    lcd_obj.info.framebuffer    = (void *)lcd_obj.fb_virt;
    
    lcd_obj.parent.type = RT_Device_Class_Graphic;

#ifdef RT_USING_DEVICE_OPS
    lcd_obj.parent.ops         = &elcd_ops;
#else
    lcd_obj.parent.init    = imx6ull_elcd_init;
    lcd_obj.parent.open    = RT_NULL;
    lcd_obj.parent.close   = RT_NULL;
    lcd_obj.parent.read    = RT_NULL;
    lcd_obj.parent.write   = RT_NULL;
    lcd_obj.parent.control = imx6ull_elcd_control;
#endif

    lcd_obj.parent.user_data = (void *)&lcd_obj.info;

    ret = rt_device_register(&lcd_obj.parent, lcd_obj.config->name, RT_DEVICE_FLAG_RDWR);

    /* LCD_BL */
    rt_pin_mode (IMX6ULL_LCD_BL_PIN, PIN_MODE_OUTPUT);  
    rt_pin_write(IMX6ULL_LCD_BL_PIN, PIN_HIGH);

    rt_memset((rt_uint8_t *)lcd_obj.fb_virt, 0xff, LCD_BUF_SIZE);

    return ret;
}
INIT_DEVICE_EXPORT(rt_hw_elcd_init);

#endif

