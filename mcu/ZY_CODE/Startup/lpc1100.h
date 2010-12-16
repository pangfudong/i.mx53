/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:            LPC1100.H
** Last modified Date:   2009-05-12
** Last Version:         V1.01
** Descriptions:         lpc1100寄存器地址定义
**
**--------------------------------------------------------------------------------------------------------
** Created by:           Liangbaoqiong
** Created date:         2009-05-12
** Version:              V1.00
** Descriptions:         
**--------------------------------------------------------------------------------------------------------
** Modified by:          Lanwuqiang
** Modified date:        2009-10-21
** Version:              V1.01
** Descriptions:         定义LPC1100寄存器
*********************************************************************************************************/
#ifndef __IOLPC1100_H
#define __IOLPC1100_H

/*********************************************************************************************************
  NVIC嵌套向量中断控制器
*********************************************************************************************************/
#define NVIC                    (*(volatile unsigned long *)0xE000E004)
#define STCTRL                  (*(volatile unsigned long *)0xE000E010)
#define STRELOAD                (*(volatile unsigned long *)0xE000E014)
#define STCURR                  (*(volatile unsigned long *)0xE000E018)
#define STCALIB                 (*(volatile unsigned long *)0xE000E01C)
#define ISER                    (*(volatile unsigned long *)0xE000E100)
#define SETENA                  (*(volatile unsigned long *)0xE000E100)
#define ICER                    (*(volatile unsigned long *)0xE000E180)
#define CLRENA                  (*(volatile unsigned long *)0xE000E180)
#define ISPR                    (*(volatile unsigned long *)0xE000E200)
#define SETPEND                 (*(volatile unsigned long *)0xE000E200)
#define ICPR                    (*(volatile unsigned long *)0xE000E280)
#define CLRPEND                 (*(volatile unsigned long *)0xE000E280)
#define IP0                     (*(volatile unsigned long *)0xE000E400)
#define IP1                     (*(volatile unsigned long *)0xE000E404)
#define IP2                     (*(volatile unsigned long *)0xE000E408)
#define IP3                     (*(volatile unsigned long *)0xE000E40C)
#define IP4                     (*(volatile unsigned long *)0xE000E410)
#define IP5                     (*(volatile unsigned long *)0xE000E414)
#define IP6                     (*(volatile unsigned long *)0xE000E418)
#define IP7                     (*(volatile unsigned long *)0xE000E41C)

#define CPUIDBR                 (*(volatile unsigned long *)0xE000ED00)
#define ICSR                    (*(volatile unsigned long *)0xE000ED04)
#define VTOR                    (*(volatile unsigned long *)0xE000ED08)
#define AITCR                   (*(volatile unsigned long *)0xE000ED0C)
#define SCR                     (*(volatile unsigned long *)0xE000ED10)
#define CCR                     (*(volatile unsigned long *)0xE000ED14)
#define SHPR0                   (*(volatile unsigned long *)0xE000ED18)
#define SHPR1                   (*(volatile unsigned long *)0xE000ED1C)
#define SHPR2                   (*(volatile unsigned long *)0xE000ED20)
#define SHCSR                   (*(volatile unsigned long *)0xE000ED24)
#define CFSR                    (*(volatile unsigned long *)0xE000ED28)
#define HFSR                    (*(volatile unsigned long *)0xE000ED2C)
#define DFSR                    (*(volatile unsigned long *)0xE000ED30)
#define MMFAR                   (*(volatile unsigned long *)0xE000ED34)
#define BFAR                    (*(volatile unsigned long *)0xE000ED38)
#define STIR                    (*(volatile unsigned long *)0xE000EF00)

/*********************************************************************************************************
  Base addresses
*********************************************************************************************************/
#define FLASH_BASE              (0x00000000UL)
#define RAM_BASE                (0x10000000UL)
#define APB0_BASE               (0x40000000UL)
#define AHB_BASE                (0x50000000UL)

/*********************************************************************************************************
  I2C
*********************************************************************************************************/
#define I2C_BASE                (APB0_BASE + 0x00000)

#define I2CCONSET               (*(volatile unsigned long *)(I2C_BASE + 0x000))
#define I2CSTAT                 (*(volatile unsigned long *)(I2C_BASE + 0x004))
#define I2CDAT                  (*(volatile unsigned long *)(I2C_BASE + 0x008))
#define I2CADR0                 (*(volatile unsigned long *)(I2C_BASE + 0x00C))
#define I2CSCLH                 (*(volatile unsigned long *)(I2C_BASE + 0x010))
#define I2CSCLL                 (*(volatile unsigned long *)(I2C_BASE + 0x014))
#define I2CCONCLR               (*(volatile unsigned long *)(I2C_BASE + 0x018))
#define I2CMMCTRL               (*(volatile unsigned long *)(I2C_BASE + 0x01C))
#define I2CADR1                 (*(volatile unsigned long *)(I2C_BASE + 0x020))
#define I2CADR2                 (*(volatile unsigned long *)(I2C_BASE + 0x024))
#define I2CADR3                 (*(volatile unsigned long *)(I2C_BASE + 0x028))
#define I2CDATABUFFER           (*(volatile unsigned long *)(I2C_BASE + 0x02C))
#define I2CMASK0                (*(volatile unsigned long *)(I2C_BASE + 0x030))
#define I2CMASK1                (*(volatile unsigned long *)(I2C_BASE + 0x034))
#define I2CMASK2                (*(volatile unsigned long *)(I2C_BASE + 0x038))
#define I2CMASK3                (*(volatile unsigned long *)(I2C_BASE + 0x03C))

/*********************************************************************************************************
  WatchDogTimer
*********************************************************************************************************/
#define WDT_BASE                (APB0_BASE + 0x04000)

#define WDTMOD                  (*(volatile unsigned long *)(WDT_BASE + 0x000))
#define WDTTC                   (*(volatile unsigned long *)(WDT_BASE + 0x004))
#define WDTFEED                 (*(volatile unsigned long *)(WDT_BASE + 0x008))
#define WDTTV                   (*(volatile unsigned long *)(WDT_BASE + 0x00C))

/*********************************************************************************************************
  UART
*********************************************************************************************************/
#define UART_BASE               (APB0_BASE + 0x08000)

#define U0RBR                   (*(volatile unsigned long *)(UART_BASE + 0x000))
#define U0THR                   (*(volatile unsigned long *)(UART_BASE + 0x000))
#define U0DLL                   (*(volatile unsigned long *)(UART_BASE + 0x000))
#define U0DLM                   (*(volatile unsigned long *)(UART_BASE + 0x004))
#define U0IER                   (*(volatile unsigned long *)(UART_BASE + 0x004))
#define U0IIR                   (*(volatile unsigned long *)(UART_BASE + 0x008))
#define U0FCR                   (*(volatile unsigned long *)(UART_BASE + 0x008))
#define U0LCR                   (*(volatile unsigned long *)(UART_BASE + 0x00C))
#define U0MCR                   (*(volatile unsigned long *)(UART_BASE + 0x010))
#define U0LSR                   (*(volatile unsigned long *)(UART_BASE + 0x014))
#define U0MSR                   (*(volatile unsigned long *)(UART_BASE + 0x018))
#define U0SCR                   (*(volatile unsigned long *)(UART_BASE + 0x01C))
#define U0ACR                   (*(volatile unsigned long *)(UART_BASE + 0x020))
#define U0FDR                   (*(volatile unsigned long *)(UART_BASE + 0x028))
#define U0TER                   (*(volatile unsigned long *)(UART_BASE + 0x030))
#define U0RS485CTRL             (*(volatile unsigned long *)(UART_BASE + 0x04C))
#define U0ADRMATCH              (*(volatile unsigned long *)(UART_BASE + 0x050))
#define U0RS485DLY              (*(volatile unsigned long *)(UART_BASE + 0x054))
#define U0FIFOLVL               (*(volatile unsigned long *)(UART_BASE + 0x058))

/*********************************************************************************************************
  16bit Counter/Timer 0
*********************************************************************************************************/
#define CT16B0_BASE             (APB0_BASE + 0x0C000)

#define TMR16B0IR               (*(volatile unsigned long *)(CT16B0_BASE + 0x000))
#define TMR16B0TCR              (*(volatile unsigned long *)(CT16B0_BASE + 0x004))
#define TMR16B0TC               (*(volatile unsigned long *)(CT16B0_BASE + 0x008))
#define TMR16B0PR               (*(volatile unsigned long *)(CT16B0_BASE + 0x00C))
#define TMR16B0PC               (*(volatile unsigned long *)(CT16B0_BASE + 0x010))
#define TMR16B0MCR              (*(volatile unsigned long *)(CT16B0_BASE + 0x014))
#define TMR16B0MR0              (*(volatile unsigned long *)(CT16B0_BASE + 0x018))
#define TMR16B0MR1              (*(volatile unsigned long *)(CT16B0_BASE + 0x01C))
#define TMR16B0MR2              (*(volatile unsigned long *)(CT16B0_BASE + 0x020))
#define TMR16B0MR3              (*(volatile unsigned long *)(CT16B0_BASE + 0x024))
#define TMR16B0CCR              (*(volatile unsigned long *)(CT16B0_BASE + 0x028))
#define TMR16B0CR0              (*(volatile unsigned long *)(CT16B0_BASE + 0x02C))
#define TMR16B0EMR              (*(volatile unsigned long *)(CT16B0_BASE + 0x03C))
#define TMR16B0CTCR             (*(volatile unsigned long *)(CT16B0_BASE + 0x070))
#define TMR16B0PWMC             (*(volatile unsigned long *)(CT16B0_BASE + 0x074))

/*********************************************************************************************************
  16 BIT  Counter/Timer 1
*********************************************************************************************************/
#define CT16B1_BASE             (APB0_BASE + 0x10000)

#define TMR16B1IR               (*(volatile unsigned long *)(CT16B1_BASE + 0x000))
#define TMR16B1TCR              (*(volatile unsigned long *)(CT16B1_BASE + 0x004))
#define TMR16B1TC               (*(volatile unsigned long *)(CT16B1_BASE + 0x008))
#define TMR16B1PR               (*(volatile unsigned long *)(CT16B1_BASE + 0x00C))
#define TMR16B1PC               (*(volatile unsigned long *)(CT16B1_BASE + 0x010))
#define TMR16B1MCR              (*(volatile unsigned long *)(CT16B1_BASE + 0x014))
#define TMR16B1MR0              (*(volatile unsigned long *)(CT16B1_BASE + 0x018))
#define TMR16B1MR1              (*(volatile unsigned long *)(CT16B1_BASE + 0x01C))
#define TMR16B1MR2              (*(volatile unsigned long *)(CT16B1_BASE + 0x020))
#define TMR16B1MR3              (*(volatile unsigned long *)(CT16B1_BASE + 0x024))
#define TMR16B1CCR              (*(volatile unsigned long *)(CT16B1_BASE + 0x028))
#define TMR16B1CR0              (*(volatile unsigned long *)(CT16B1_BASE + 0x02C))
#define TMR16B1EMR              (*(volatile unsigned long *)(CT16B1_BASE + 0x03C))
#define TMR16B1CTCR             (*(volatile unsigned long *)(CT16B1_BASE + 0x070))
#define TMR16B1PWMC             (*(volatile unsigned long *)(CT16B1_BASE + 0x074))

/*********************************************************************************************************
  32 bit Counter/Timer 0
*********************************************************************************************************/
#define CT32B0_BASE             (APB0_BASE + 0x14000)

#define TMR32B0IR               (*(volatile unsigned long *)(CT32B0_BASE + 0x000))
#define TMR32B0TCR              (*(volatile unsigned long *)(CT32B0_BASE + 0x004))
#define TMR32B0TC               (*(volatile unsigned long *)(CT32B0_BASE + 0x008))
#define TMR32B0PR               (*(volatile unsigned long *)(CT32B0_BASE + 0x00C))
#define TMR32B0PC               (*(volatile unsigned long *)(CT32B0_BASE + 0x010))
#define TMR32B0MCR              (*(volatile unsigned long *)(CT32B0_BASE + 0x014))
#define TMR32B0MR0              (*(volatile unsigned long *)(CT32B0_BASE + 0x018))
#define TMR32B0MR1              (*(volatile unsigned long *)(CT32B0_BASE + 0x01C))
#define TMR32B0MR2              (*(volatile unsigned long *)(CT32B0_BASE + 0x020))
#define TMR32B0MR3              (*(volatile unsigned long *)(CT32B0_BASE + 0x024))
#define TMR32B0CCR              (*(volatile unsigned long *)(CT32B0_BASE + 0x028))
#define TMR32B0CR0              (*(volatile unsigned long *)(CT32B0_BASE + 0x02C))
#define TMR32B0EMR              (*(volatile unsigned long *)(CT32B0_BASE + 0x03C))
#define TMR32B0CTCR             (*(volatile unsigned long *)(CT32B0_BASE + 0x070))
#define TMR32B0PWMC             (*(volatile unsigned long *)(CT32B0_BASE + 0x074))

/*********************************************************************************************************
  32 bit Counter/Timer 1
*********************************************************************************************************/
#define CT32B1_BASE             (APB0_BASE + 0x18000)

#define TMR32B1IR               (*(volatile unsigned long *)(CT32B1_BASE + 0x000))
#define TMR32B1TCR              (*(volatile unsigned long *)(CT32B1_BASE + 0x004))
#define TMR32B1TC               (*(volatile unsigned long *)(CT32B1_BASE + 0x008))
#define TMR32B1PR               (*(volatile unsigned long *)(CT32B1_BASE + 0x00C))
#define TMR32B1PC               (*(volatile unsigned long *)(CT32B1_BASE + 0x010))
#define TMR32B1MCR              (*(volatile unsigned long *)(CT32B1_BASE + 0x014))
#define TMR32B1MR0              (*(volatile unsigned long *)(CT32B1_BASE + 0x018))
#define TMR32B1MR1              (*(volatile unsigned long *)(CT32B1_BASE + 0x01C))
#define TMR32B1MR2              (*(volatile unsigned long *)(CT32B1_BASE + 0x020))
#define TMR32B1MR3              (*(volatile unsigned long *)(CT32B1_BASE + 0x024))
#define TMR32B1CCR              (*(volatile unsigned long *)(CT32B1_BASE + 0x028))
#define TMR32B1CR0              (*(volatile unsigned long *)(CT32B1_BASE + 0x02C))
#define TMR32B1EMR              (*(volatile unsigned long *)(CT32B1_BASE + 0x03C))
#define TMR32B1CTCR             (*(volatile unsigned long *)(CT32B1_BASE + 0x070))
#define TMR32B1PWMC             (*(volatile unsigned long *)(CT32B1_BASE + 0x074))

/*********************************************************************************************************
  ADC
**********************************************************************************************************/
#define ADC_BASE                (APB0_BASE + 0x1C000)

#define AD0CR                   (*(volatile unsigned long *)(ADC_BASE + 0x000))
#define AD0GDR                  (*(volatile unsigned long *)(ADC_BASE + 0x004))
#define AD0INTEN                (*(volatile unsigned long *)(ADC_BASE + 0x00C))
#define AD0DR0                  (*(volatile unsigned long *)(ADC_BASE + 0x010))
#define AD0DR1                  (*(volatile unsigned long *)(ADC_BASE + 0x014))
#define AD0DR2                  (*(volatile unsigned long *)(ADC_BASE + 0x018))
#define AD0DR3                  (*(volatile unsigned long *)(ADC_BASE + 0x01C))
#define AD0DR4                  (*(volatile unsigned long *)(ADC_BASE + 0x020))
#define AD0DR5                  (*(volatile unsigned long *)(ADC_BASE + 0x024))
#define AD0DR6                  (*(volatile unsigned long *)(ADC_BASE + 0x028))
#define AD0DR7                  (*(volatile unsigned long *)(ADC_BASE + 0x02C))
#define AD0STAT                 (*(volatile unsigned long *)(ADC_BASE + 0x030))

/*********************************************************************************************************
  power management unit (PMU)
*********************************************************************************************************/
#define PMU_BASE                (APB0_BASE + 0x38000)

#define PCON                    (*(volatile unsigned long *)(PMU_BASE + 0x000))
#define GPREG0                  (*(volatile unsigned long *)(PMU_BASE + 0x004))
#define GPREG1                  (*(volatile unsigned long *)(PMU_BASE + 0x008))
#define GPREG2                  (*(volatile unsigned long *)(PMU_BASE + 0x00C))
#define GPREG3                  (*(volatile unsigned long *)(PMU_BASE + 0x010))
#define GPREG4                  (*(volatile unsigned long *)(PMU_BASE + 0x014))

/*********************************************************************************************************
  Synchronous Serial Communication (SSP0)
*********************************************************************************************************/
#define SSP0_BASE               (APB0_BASE + 0x40000)

#define SSP0CR0                 (*(volatile unsigned long *)(SSP0_BASE + 0x000))
#define SSP0CR1                 (*(volatile unsigned long *)(SSP0_BASE + 0x004))
#define SSP0DR                  (*(volatile unsigned long *)(SSP0_BASE + 0x008))
#define SSP0SR                  (*(volatile unsigned long *)(SSP0_BASE + 0x00C))
#define SSP0CPSR                (*(volatile unsigned long *)(SSP0_BASE + 0x010))
#define SSP0IMSC                (*(volatile unsigned long *)(SSP0_BASE + 0x014))
#define SSP0RIS                 (*(volatile unsigned long *)(SSP0_BASE + 0x018))
#define SSP0MIS                 (*(volatile unsigned long *)(SSP0_BASE + 0x01C))
#define SSP0ICR                 (*(volatile unsigned long *)(SSP0_BASE + 0x020))

/*********************************************************************************************************
  Synchronous Serial Communication (SSP1) 仅LQFP48和PLCC44封装具有
*********************************************************************************************************/
#define SSP1_BASE               (APB0_BASE + 0x58000)

#define SSP1CR0                 (*(volatile unsigned long *)(SSP1_BASE + 0x000))
#define SSP1CR1                 (*(volatile unsigned long *)(SSP1_BASE + 0x004))
#define SSP1DR                  (*(volatile unsigned long *)(SSP1_BASE + 0x008))
#define SSP1SR                  (*(volatile unsigned long *)(SSP1_BASE + 0x00C))
#define SSP1CPSR                (*(volatile unsigned long *)(SSP1_BASE + 0x010))
#define SSP1IMSC                (*(volatile unsigned long *)(SSP1_BASE + 0x014))
#define SSP1RIS                 (*(volatile unsigned long *)(SSP1_BASE + 0x018))
#define SSP1MIS                 (*(volatile unsigned long *)(SSP1_BASE + 0x01C))
#define SSP1ICR                 (*(volatile unsigned long *)(SSP1_BASE + 0x020))

/*********************************************************************************************************
  IO Configuration Block
*********************************************************************************************************/
#define IOCON_BASE              (APB0_BASE + 0x44000)

#define IOCON_PIO2_6            (*(volatile unsigned long *)(IOCON_BASE + 0x000))
#define IOCON_PIO2_0            (*(volatile unsigned long *)(IOCON_BASE + 0x008))
#define IOCON_RESET_PIO0_0      (*(volatile unsigned long *)(IOCON_BASE + 0x00C))
#define IOCON_PIO0_1            (*(volatile unsigned long *)(IOCON_BASE + 0x010))
#define IOCON_PIO1_8            (*(volatile unsigned long *)(IOCON_BASE + 0x014))
#define IOCON_PIO0_2            (*(volatile unsigned long *)(IOCON_BASE + 0x01C))
#define IOCON_PIO2_7            (*(volatile unsigned long *)(IOCON_BASE + 0x020))
#define IOCON_PIO2_8            (*(volatile unsigned long *)(IOCON_BASE + 0x024))
#define IOCON_PIO2_1            (*(volatile unsigned long *)(IOCON_BASE + 0x028))
#define IOCON_PIO0_3            (*(volatile unsigned long *)(IOCON_BASE + 0x02C))
#define IOCON_PIO0_4            (*(volatile unsigned long *)(IOCON_BASE + 0x030))
#define IOCON_PIO0_5            (*(volatile unsigned long *)(IOCON_BASE + 0x034))
#define IOCON_PIO1_9            (*(volatile unsigned long *)(IOCON_BASE + 0x038))
#define IOCON_PIO3_4            (*(volatile unsigned long *)(IOCON_BASE + 0x03C))
#define IOCON_PIO2_4            (*(volatile unsigned long *)(IOCON_BASE + 0x040))

#define IOCON_PIO2_5            (*(volatile unsigned long *)(IOCON_BASE + 0x044))
#define IOCON_PIO3_5            (*(volatile unsigned long *)(IOCON_BASE + 0x048))
#define IOCON_PIO0_6            (*(volatile unsigned long *)(IOCON_BASE + 0x04C))
#define IOCON_PIO0_7            (*(volatile unsigned long *)(IOCON_BASE + 0x050))
#define IOCON_PIO2_9            (*(volatile unsigned long *)(IOCON_BASE + 0x054))
#define IOCON_PIO2_10           (*(volatile unsigned long *)(IOCON_BASE + 0x058))
#define IOCON_PIO2_2            (*(volatile unsigned long *)(IOCON_BASE + 0x05C))

#define IOCON_PIO0_8            (*(volatile unsigned long *)(IOCON_BASE + 0x060))
#define IOCON_PIO0_9            (*(volatile unsigned long *)(IOCON_BASE + 0x064))
#define IOCON_JTAG_TCK_PIO0_10  (*(volatile unsigned long *)(IOCON_BASE + 0x068))
#define IOCON_PIO1_10           (*(volatile unsigned long *)(IOCON_BASE + 0x06C))
#define IOCON_PIO2_11           (*(volatile unsigned long *)(IOCON_BASE + 0x070))
#define IOCON_JTAG_TDI_PIO0_11  (*(volatile unsigned long *)(IOCON_BASE + 0x074))
#define IOCON_JTAG_TMS_PIO1_0   (*(volatile unsigned long *)(IOCON_BASE + 0x078))
#define IOCON_JTAG_TDO_PIO1_1   (*(volatile unsigned long *)(IOCON_BASE + 0x07C))

#define IOCON_JTAG_nTRST_PIO1_2 (*(volatile unsigned long *)(IOCON_BASE + 0x080))
#define IOCON_PIO3_0            (*(volatile unsigned long *)(IOCON_BASE + 0x084))
#define IOCON_PIO3_1            (*(volatile unsigned long *)(IOCON_BASE + 0x088))
#define IOCON_PIO2_3            (*(volatile unsigned long *)(IOCON_BASE + 0x08C))
#define IOCON_ARM_SWDIO_PIO1_3  (*(volatile unsigned long *)(IOCON_BASE + 0x090))
#define IOCON_PIO1_4            (*(volatile unsigned long *)(IOCON_BASE + 0x094))
#define IOCON_PIO1_11           (*(volatile unsigned long *)(IOCON_BASE + 0x098))
#define IOCON_PIO3_2            (*(volatile unsigned long *)(IOCON_BASE + 0x09C))

#define IOCON_PIO1_5            (*(volatile unsigned long *)(IOCON_BASE + 0x0A0))
#define IOCON_PIO1_6            (*(volatile unsigned long *)(IOCON_BASE + 0x0A4))
#define IOCON_PIO1_7            (*(volatile unsigned long *)(IOCON_BASE + 0x0A8))
#define IOCON_PIO3_3            (*(volatile unsigned long *)(IOCON_BASE + 0x0AC))

#define IOCON_SCKLOC            (*(volatile unsigned long *)(IOCON_BASE + 0x110))

/*********************************************************************************************************
  ICON_PIO
*********************************************************************************************************/
#define IOCON_PIO0_0            (*(volatile unsigned long *)(IOCON_BASE + 0x00C))
#define IOCON_PIO0_1            (*(volatile unsigned long *)(IOCON_BASE + 0x010))
#define IOCON_PIO0_2            (*(volatile unsigned long *)(IOCON_BASE + 0x01C))
#define IOCON_PIO0_3            (*(volatile unsigned long *)(IOCON_BASE + 0x02C))
#define IOCON_PIO0_4            (*(volatile unsigned long *)(IOCON_BASE + 0x030))
#define IOCON_PIO0_5            (*(volatile unsigned long *)(IOCON_BASE + 0x034))
#define IOCON_PIO0_6            (*(volatile unsigned long *)(IOCON_BASE + 0x04C))
#define IOCON_PIO0_7            (*(volatile unsigned long *)(IOCON_BASE + 0x050))
#define IOCON_PIO0_8            (*(volatile unsigned long *)(IOCON_BASE + 0x060))
#define IOCON_PIO0_9            (*(volatile unsigned long *)(IOCON_BASE + 0x064))
#define IOCON_PIO0_10           (*(volatile unsigned long *)(IOCON_BASE + 0x068))
#define IOCON_PIO0_11           (*(volatile unsigned long *)(IOCON_BASE + 0x074))

#define IOCON_PIO1_0            (*(volatile unsigned long *)(IOCON_BASE + 0x078))
#define IOCON_PIO1_1            (*(volatile unsigned long *)(IOCON_BASE + 0x07C))
#define IOCON_PIO1_2            (*(volatile unsigned long *)(IOCON_BASE + 0x080))
#define IOCON_PIO1_3            (*(volatile unsigned long *)(IOCON_BASE + 0x090))
#define IOCON_PIO1_4            (*(volatile unsigned long *)(IOCON_BASE + 0x094))
#define IOCON_PIO1_5            (*(volatile unsigned long *)(IOCON_BASE + 0x0A0))
#define IOCON_PIO1_6            (*(volatile unsigned long *)(IOCON_BASE + 0x0A4))
#define IOCON_PIO1_7            (*(volatile unsigned long *)(IOCON_BASE + 0x0A8))
#define IOCON_PIO1_8            (*(volatile unsigned long *)(IOCON_BASE + 0x014))
#define IOCON_PIO1_9            (*(volatile unsigned long *)(IOCON_BASE + 0x038))
#define IOCON_PIO1_10           (*(volatile unsigned long *)(IOCON_BASE + 0x06C))
#define IOCON_PIO1_11           (*(volatile unsigned long *)(IOCON_BASE + 0x098))

#define IOCON_PIO2_0            (*(volatile unsigned long *)(IOCON_BASE + 0x008))
#define IOCON_PIO2_1            (*(volatile unsigned long *)(IOCON_BASE + 0x028))
#define IOCON_PIO2_2            (*(volatile unsigned long *)(IOCON_BASE + 0x05C))
#define IOCON_PIO2_3            (*(volatile unsigned long *)(IOCON_BASE + 0x08C))
#define IOCON_PIO2_4            (*(volatile unsigned long *)(IOCON_BASE + 0x040))
#define IOCON_PIO2_5            (*(volatile unsigned long *)(IOCON_BASE + 0x044))
#define IOCON_PIO2_6            (*(volatile unsigned long *)(IOCON_BASE + 0x000))
#define IOCON_PIO2_7            (*(volatile unsigned long *)(IOCON_BASE + 0x020))
#define IOCON_PIO2_8            (*(volatile unsigned long *)(IOCON_BASE + 0x024))
#define IOCON_PIO2_9            (*(volatile unsigned long *)(IOCON_BASE + 0x054))
#define IOCON_PIO2_10           (*(volatile unsigned long *)(IOCON_BASE + 0x058))
#define IOCON_PIO2_11           (*(volatile unsigned long *)(IOCON_BASE + 0x070))

#define IOCON_PIO3_0            (*(volatile unsigned long *)(IOCON_BASE + 0x084))
#define IOCON_PIO3_1            (*(volatile unsigned long *)(IOCON_BASE + 0x088))
#define IOCON_PIO3_2            (*(volatile unsigned long *)(IOCON_BASE + 0x09C))
#define IOCON_PIO3_3            (*(volatile unsigned long *)(IOCON_BASE + 0x0AC))
#define IOCON_PIO3_4            (*(volatile unsigned long *)(IOCON_BASE + 0x03C))
#define IOCON_PIO3_5            (*(volatile unsigned long *)(IOCON_BASE + 0x048))

/*********************************************************************************************************
  System Control
*********************************************************************************************************/
#define SYSCON_BASE             (APB0_BASE + 0x48000)

#define SYSMEMREMAP             (*(volatile unsigned long *)(SYSCON_BASE + 0x000))
#define PRESETCTRL              (*(volatile unsigned long *)(SYSCON_BASE + 0x004))
#define SYSPLLCTRL              (*(volatile unsigned long *)(SYSCON_BASE + 0x008))
#define SYSPLLSTAT              (*(volatile unsigned long *)(SYSCON_BASE + 0x00C))
#define USBPLLCTRL              (*(volatile unsigned long *)(SYSCON_BASE + 0x010))
#define USBPLLSTAT              (*(volatile unsigned long *)(SYSCON_BASE + 0x014))
#define SYSOSCCTRL              (*(volatile unsigned long *)(SYSCON_BASE + 0x020))
#define WDTOSCCTRL              (*(volatile unsigned long *)(SYSCON_BASE + 0x024))
#define IRCCTRL                 (*(volatile unsigned long *)(SYSCON_BASE + 0x028))
#define SYSRESSTAT              (*(volatile unsigned long *)(SYSCON_BASE + 0x030))
#define SYSPLLCLKSEL            (*(volatile unsigned long *)(SYSCON_BASE + 0x040))
#define SYSPLLCLKUEN            (*(volatile unsigned long *)(SYSCON_BASE + 0x044))
#define USBPLLCLKSEL            (*(volatile unsigned long *)(SYSCON_BASE + 0x048))
#define USBPLLCLKUEN            (*(volatile unsigned long *)(SYSCON_BASE + 0x04C))
#define MAINCLKSEL              (*(volatile unsigned long *)(SYSCON_BASE + 0x070))
#define MAINCLKUEN              (*(volatile unsigned long *)(SYSCON_BASE + 0x074))
#define SYSAHBCLKDIV            (*(volatile unsigned long *)(SYSCON_BASE + 0x078))
#define SYSAHBCLKCTRL           (*(volatile unsigned long *)(SYSCON_BASE + 0x080))
#define SSP0CLKDIV              (*(volatile unsigned long *)(SYSCON_BASE + 0x094))
#define UARTCLKDIV              (*(volatile unsigned long *)(SYSCON_BASE + 0x098))
#define SSP1CLKDIV              (*(volatile unsigned long *)(SYSCON_BASE + 0x09C))
#define SYSTICKCLKDIV           (*(volatile unsigned long *)(SYSCON_BASE + 0x0B0))
#define USBCLKSEL               (*(volatile unsigned long *)(SYSCON_BASE + 0x0C0))
#define USBCLKUEN               (*(volatile unsigned long *)(SYSCON_BASE + 0x0C4))
#define USBCLKDIV               (*(volatile unsigned long *)(SYSCON_BASE + 0x0C8))
#define WDTCLKSEL               (*(volatile unsigned long *)(SYSCON_BASE + 0x0D0))
#define WDTCLKUEN               (*(volatile unsigned long *)(SYSCON_BASE + 0x0D4))
#define WDTCLKDIV               (*(volatile unsigned long *)(SYSCON_BASE + 0x0D8))
#define CLKOUTCLKSEL            (*(volatile unsigned long *)(SYSCON_BASE + 0x0E0))
#define CLKOUTUEN               (*(volatile unsigned long *)(SYSCON_BASE + 0x0E4))
#define CLKOUTDIV               (*(volatile unsigned long *)(SYSCON_BASE + 0x0E8))
#define PIOPORCAP0              (*(volatile unsigned long *)(SYSCON_BASE + 0x100))
#define PIOPORCAP1              (*(volatile unsigned long *)(SYSCON_BASE + 0x104))
#define BODCTRL                 (*(volatile unsigned long *)(SYSCON_BASE + 0x150))
#define SYSTCKCAL               (*(volatile unsigned long *)(SYSCON_BASE + 0x158))
#define STARTAPRP0              (*(volatile unsigned long *)(SYSCON_BASE + 0x200))
#define STARTERP0               (*(volatile unsigned long *)(SYSCON_BASE + 0x204))
#define STARTRSRP0CLR           (*(volatile unsigned long *)(SYSCON_BASE + 0x208))
#define STARTSRP0               (*(volatile unsigned long *)(SYSCON_BASE + 0x20C))
#define STARTAPRP1              (*(volatile unsigned long *)(SYSCON_BASE + 0x210))
#define STARTERP1               (*(volatile unsigned long *)(SYSCON_BASE + 0x214))
#define STARTRSRP1CLR           (*(volatile unsigned long *)(SYSCON_BASE + 0x218))
#define STARTSRP1               (*(volatile unsigned long *)(SYSCON_BASE + 0x21C))
#define PDSLEEPCFG              (*(volatile unsigned long *)(SYSCON_BASE + 0x230))
#define PDAWAKECFG              (*(volatile unsigned long *)(SYSCON_BASE + 0x234))
#define PDRUNCFG                (*(volatile unsigned long *)(SYSCON_BASE + 0x238))
#define DEVICE_ID               (*(volatile unsigned long *)(SYSCON_BASE + 0x3F4))

/*********************************************************************************************************
  AHB peripherals
*********************************************************************************************************/
#define GPIO_BASE               (AHB_BASE  + 0x00000)
#define GPIO0DATA               (*(volatile unsigned long *)(GPIO_BASE + 0x00000 + 0x3FFC))
#define GPIO0MASKED_ACCESS(bit) (*(volatile unsigned long *)(GPIO_BASE + 0x00000 + (bit)))
#define GPIO1DATA               (*(volatile unsigned long *)(GPIO_BASE + 0x10000 + 0x3FFC))
#define GPIO1MASKED_ACCESS(bit) (*(volatile unsigned long *)(GPIO_BASE + 0x10000 + (bit)))
#define GPIO2DATA               (*(volatile unsigned long *)(GPIO_BASE + 0x20000 + 0x3FFC))
#define GPIO2MASKED_ACCESS(bit) (*(volatile unsigned long *)(GPIO_BASE + 0x20000 + (bit)))
#define GPIO3DATA               (*(volatile unsigned long *)(GPIO_BASE + 0x30000 + 0x3FFC))
#define GPIO3MASKED_ACCESS(bit) (*(volatile unsigned long *)(GPIO_BASE + 0x30000 + (bit)))

#define GPIO0_BASE              (AHB_BASE  + 0x00000)
#define GPIO0DIR                (*(volatile unsigned long *)(GPIO0_BASE + 0x8000))
#define GPIO0IS                 (*(volatile unsigned long *)(GPIO0_BASE + 0x8004))
#define GPIO0IBE                (*(volatile unsigned long *)(GPIO0_BASE + 0x8008))
#define GPIO0IEV                (*(volatile unsigned long *)(GPIO0_BASE + 0x800C))
#define GPIO0IE                 (*(volatile unsigned long *)(GPIO0_BASE + 0x8010))
#define GPIO0RIS                (*(volatile unsigned long *)(GPIO0_BASE + 0x8014))
#define GPIO0MIS                (*(volatile unsigned long *)(GPIO0_BASE + 0x8018))
#define GPIO0IC                 (*(volatile unsigned long *)(GPIO0_BASE + 0x801C))

#define GPIO1_BASE              (AHB_BASE  + 0x10000)
#define GPIO1DIR                (*(volatile unsigned long *)(GPIO1_BASE + 0x8000))
#define GPIO1IS                 (*(volatile unsigned long *)(GPIO1_BASE + 0x8004))
#define GPIO1IBE                (*(volatile unsigned long *)(GPIO1_BASE + 0x8008))
#define GPIO1IEV                (*(volatile unsigned long *)(GPIO1_BASE + 0x800C))
#define GPIO1IE                 (*(volatile unsigned long *)(GPIO1_BASE + 0x8010))
#define GPIO1RIS                (*(volatile unsigned long *)(GPIO1_BASE + 0x8014))
#define GPIO1MIS                (*(volatile unsigned long *)(GPIO1_BASE + 0x8018))
#define GPIO1IC                 (*(volatile unsigned long *)(GPIO1_BASE + 0x801C))

#define GPIO2_BASE              (AHB_BASE  + 0x20000)
#define GPIO2DIR                (*(volatile unsigned long *)(GPIO2_BASE + 0x8000))
#define GPIO2IS                 (*(volatile unsigned long *)(GPIO2_BASE + 0x8004))
#define GPIO2IBE                (*(volatile unsigned long *)(GPIO2_BASE + 0x8008))
#define GPIO2IEV                (*(volatile unsigned long *)(GPIO2_BASE + 0x800C))
#define GPIO2IE                 (*(volatile unsigned long *)(GPIO2_BASE + 0x8010))
#define GPIO2RIS                (*(volatile unsigned long *)(GPIO2_BASE + 0x8014))
#define GPIO2MIS                (*(volatile unsigned long *)(GPIO2_BASE + 0x8018))
#define GPIO2IC                 (*(volatile unsigned long *)(GPIO2_BASE + 0x801C))

#define GPIO3_BASE              (AHB_BASE  + 0x30000)
#define GPIO3DIR                (*(volatile unsigned long *)(GPIO3_BASE + 0x8000))
#define GPIO3IS                 (*(volatile unsigned long *)(GPIO3_BASE + 0x8004))
#define GPIO3IBE                (*(volatile unsigned long *)(GPIO3_BASE + 0x8008))
#define GPIO3IEV                (*(volatile unsigned long *)(GPIO3_BASE + 0x800C))
#define GPIO3IE                 (*(volatile unsigned long *)(GPIO3_BASE + 0x8010))
#define GPIO3RIS                (*(volatile unsigned long *)(GPIO3_BASE + 0x8014))
#define GPIO3MIS                (*(volatile unsigned long *)(GPIO3_BASE + 0x8018))
#define GPIO3IC                 (*(volatile unsigned long *)(GPIO3_BASE + 0x801C))

#define ST_BASE                 (0xE000E000+ 0x0010)
#define SYSTICKCTRL             (*(volatile unsigned long *)(SYSTICK_BASE + 0x0000))
#define SYSTICKLOAD             (*(volatile unsigned long *)(SYSTICK_BASE + 0x0004))
#define SYSTICKVAL              (*(volatile unsigned long *)(SYSTICK_BASE + 0x0008))
#define SYSTICKCALIB            (*(volatile unsigned long *)(SYSTICK_BASE + 0x000C))

/*********************************************************************************************************
  NVIC Interrupt channels  NVIC中断控制通道号
*********************************************************************************************************/
#define MAIN_STACK              0                                       /*  Main Stack                  */
#define RESETI                  1                                       /*  Reset                       */
#define NMII                    2                                       /*  Non-maskable Interrupt      */
#define HFI                     3                                       /*  Hard Fault                  */
#define MMI                     4                                       /*  Memory Management           */
#define BFI                     5                                       /*  Bus Fault                   */
#define UFI                     6                                       /*  Usage Fault                 */
#define SVCI                    11                                      /*  SVCall                      */
#define DMI                     12                                      /*  Debug Monitor               */
#define PSI                     14                                      /*  PendSV                      */
#define STI                     15                                      /*  SysTick                     */
#define WAKEUP_PIO0_0           16                                      /*  PI0_0 Interup wake-up       */
#define WAKEUP_PIO0_1           17                                      /*  PI0_1 Interup wake-up       */
#define WAKEUP_PIO0_2           18                                      /*  PI0_2 Interup wake-up       */
#define WAKEUP_PIO0_3           19                                      /*  PI0_3 Interup wake-up       */
#define WAKEUP_PIO0_4           20                                      /*  PI0_4 Interup wake-up       */
#define WAKEUP_PIO0_5           21                                      /*  PI0_5 Interup wake-up       */
#define WAKEUP_PIO0_6           22                                      /*  PI0_6 Interup wake-up       */
#define WAKEUP_PIO0_7           23                                      /*  PI0_7 Interup wake-up       */
#define WAKEUP_PIO0_8           24                                      /*  PI0_8 Interup wake-up       */
#define WAKEUP_PIO0_9           25                                      /*  PI0_9 Interup wake-up       */
#define WAKEUP_PIO0_10          26                                      /*  PI0_10 Interup wake-up      */
#define WAKEUP_PIO0_11          27                                      /*  PI0_11 Interup wake-up      */
#define WAKEUP_PIO1_0           28                                      /*  PI1_0 Interup wake-up       */
#define NVIC_SSP1               30                                      /*  TxFIFO half Empty RxFIFO    */
#define NVIC_I2C                31                                      /*  SI State Change             */
#define NVIC_TIMER16B0          32                                      /*  Match0-x caputre0-x         */
#define NVIC_TIMER16B1          33                                      /*  Match0-x capture0-x         */
#define NVIC_TIMER32B0          34                                      /*  Match0-x capture0-x         */
#define NVIC_TIMER32B1          35                                      /*  Match0-x capture0-x         */
#define NVIC_SSP0               36                                      /*  TxFIFO half Empty RxFIFO    */
#define NVIC_UART               37                                      /*  RLS,THRE,RDA,CTI,ABEO,ABTO  */
#define NVIC_ADC                40                                      /*  ADC                         */
#define NVIC_WDT                41                                      /*  WDTINT                      */
#define NVIC_BOD                42                                      /*  Brown-out detect            */
#define NVIC_PIOINT3            44                                      /*  GPIO3 Interrupt Status      */
#define NVIC_PIOINT2            45                                      /*  GPIO2 Interrupt Status      */
#define NVIC_PIOINT1            46                                      /*  GPIO1 Interrupt Status      */
#define NVIC_PIOINT0            47                                      /*  GPIO0 Interrupt Status      */
                                                                        
/*********************************************************************************************************
  NVIC Interrupt Priority   NVIC中断优先级
*********************************************************************************************************/
#define PRIO_ZERO               (0x00)                                  /*  优先级为0                   */
#define PRIO_ONE                (0x01ul << 6)                           /*  优先级为1                   */
#define PRIO_TWO                (0x02ul << 6)                           /*  优先级为2                   */
#define PRIO_THREE              (0x03ul << 6)                           /*  优先级为3                   */

#endif                                                                  /*  __IOLPC1100_H               */
/*********************************************************************************************************
  End Of File
*********************************************************************************************************/

