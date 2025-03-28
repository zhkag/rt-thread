/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-29     RealThread   first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <rtthread.h>
#include <stm32h7rsxx.h>
#include <drv_common.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*-------------------------- CHIP CONFIG BEGIN --------------------------*/

#define CHIP_FAMILY_STM32
#define CHIP_SERIES_STM32H7RS
#define CHIP_NAME_STM32H750XBHX

/*-------------------------- CHIP CONFIG END --------------------------*/

/*-------------------------- ROM/RAM CONFIG BEGIN --------------------------*/
/**
 * @brief H7RS7 SRAM MEMORY Layout
 * 0x24060000 - 0x23071FFF AXI SRAM shared with ECC
 * 0x24040000 - 0x2305FFFF AXI SRAM shared with DTCM
 * 0x24020000 - 0x2403FFFF AXI SRAM
 * 0x24000000 - 0x2401FFFF AXI SRAM shared with ITCM
 */
#define ROM_START              ((uint32_t)0x70000000)
#define ROM_SIZE               (131072)
#define ROM_END                ((uint32_t)(ROM_START + ROM_SIZE * 1024))

#define RAM_START              (0x24000000)
#define RAM_SIZE               (456)
#define RAM_END                (RAM_START + RAM_SIZE * 1024)

/*-------------------------- ROM/RAM CONFIG END --------------------------*/

/*-------------------------- CLOCK CONFIG BEGIN --------------------------*/

#define BSP_CLOCK_SOURCE                  ("HSE")
#define BSP_CLOCK_SOURCE_FREQ_MHZ         ((int32_t)0)
#define BSP_CLOCK_SYSTEM_FREQ_MHZ         ((int32_t)480)

/*-------------------------- CLOCK CONFIG END --------------------------*/

/*-------------------------- UART CONFIG BEGIN --------------------------*/

/** After configuring corresponding UART or UART DMA, you can use it.
 *
 * STEP 1, define macro define related to the serial port opening based on the serial port number
 *                 such as     #define BSP_USING_UATR1
 *
 * STEP 2, according to the corresponding pin of serial port, define the related serial port information macro
 *                 such as     #define BSP_UART1_TX_PIN       "PA9"
 *                             #define BSP_UART1_RX_PIN       "PA10"
 *
 * STEP 3, if you want using SERIAL DMA, you must open it in the RT-Thread Settings.
 *                 RT-Thread Setting -> Components -> Device Drivers -> Serial Device Drivers -> Enable Serial DMA Mode
 *
 * STEP 4, according to serial port number to define serial port tx/rx DMA function in the board.h file
 *                 such as     #define BSP_UART1_RX_USING_DMA
 *
 */

#ifdef BSP_USING_UART1
#define BSP_UART1_TX_PIN       "PA9"
#define BSP_UART1_RX_PIN       "PA10"
#endif

#ifdef BSP_USING_UART4
#define BSP_UART4_TX_PIN       "PD0"
#define BSP_UART4_RX_PIN       "PD1"
#endif


void SystemClock_Config(void);
void SystemClock_ReConfig(uint8_t mode);

/*-------------------------- UART CONFIG END --------------------------*/


#define __STM32_PORT(port)  GPIO##port##_BASE
#define GET_PIN(PORTx,PIN) (rt_base_t)((16 * ( ((rt_base_t)__STM32_PORT(PORTx) - (rt_base_t)GPIOA_BASE)/(0x0400UL) )) + PIN)
#define STM32_FLASH_START_ADRESS       ROM_START
#define STM32_FLASH_SIZE               ROM_SIZE
#define STM32_FLASH_END_ADDRESS        ROM_END

#define STM32_SRAM1_SIZE               RAM_SIZE
#define STM32_SRAM1_START              RAM_START
#define STM32_SRAM1_END                RAM_END

#define STM32_PSRAM_SIZE               ((uint32_t)32*1024*1024)
#define STM32_PSRAM_START              ((uint32_t)0x90000000)
#define STM32_PSRAM_END                ((uint32_t)(STM32_PSRAM_START + STM32_PSRAM_SIZE))

#define PSRAM_HEAP_BEGIN               STM32_PSRAM_START
#define PSRAM_HEAP_SIZE                STM32_PSRAM_SIZE
#define PSRAM_HEAP_END                 STM32_PSRAM_END

#if defined(__CC_ARM) || defined(__CLANG_ARM) || defined(__ARMCC_VERSION)
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN      ((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="CSTACK"
#define HEAP_BEGIN      (__segment_end("CSTACK"))
#else
extern int __bss_end;
#define HEAP_BEGIN      ((void *)&__bss_end)
#endif

#define HEAP_END                       STM32_SRAM1_END

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
