/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           LPC1100PinCfg.h
** Last modified Date:  2009-10-22
** Last Version:        V1.01
** Descriptions:        LPC1300系列CPU引脚连接与模式配置
**
**--------------------------------------------------------------------------------------------------------
** Created by:          Lanwuqiang
** Created date:        2009-10-22
** Version:             V1.00
** Descriptions:        
**--------------------------------------------------------------------------------------------------------
** Modified by:         ZhangNingbo
** Modified date:       2010-03-02
** Version:             V1.01
** Descriptions:        
*********************************************************************************************************/
#ifndef __LPC1100PINCFG_H
#define __LPC1100PINCFG_H

/*********************************************************************************************************
  IO口引脚功能设置宏定义
*********************************************************************************************************/

/*********************************************************************************************************
  P0_0口引脚功能配置
*********************************************************************************************************/
#define PIO0_0_RESET        0x00                                        /*  复位                        */
#define PIO0_0_GPIO         0x01                                        /*  GPIO                        */

#define FUNC_PIO0_0         PIO0_0_RESET

#define PIO0_0_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO0_0_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO0_0_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO0_0_REPEATER     0x03                                        /*  中继模式                    */

#define PIO0_0_MODE         PIO0_0_PULLUP

#define PIO0_0_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO0_0_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO0_0_HYS          PIO0_0_HYSDISABLE

#define PIO0_0_CON         (FUNC_PIO0_0 | (PIO0_0_MODE << 3) | (PIO0_0_HYS << 5))
/*********************************************************************************************************
  P0_1口引脚功能配置
*********************************************************************************************************/
#define PIO0_1_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_1_CLKOUT       0x01                                        /*  时钟输出                    */
#define PIO0_1_CT32B0_MAT2  0x02                                        /*  32位定时器0匹配输出，通道2  */

#define FUNC_PIO0_1         PIO0_1_GPIO

#define PIO0_1_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO0_1_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO0_1_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO0_1_REPEATER     0x03                                        /*  中继模式                    */

#define PIO0_1_MODE         PIO0_1_PULLUP

#define PIO0_1_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO0_1_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO0_1_HYS          PIO0_1_HYSDISABLE

#define PIO0_1_CON         (FUNC_PIO0_1 | (PIO0_1_MODE << 3) | (PIO0_1_HYS << 5))
/*********************************************************************************************************
  P0_2口引脚功能配置
*********************************************************************************************************/
#define PIO0_2_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_2_SSEL0        0x01                                        /*  SSP总线从机选择             */
#define PIO0_2_CT16B0_CAP0  0x02                                        /*  16位定时器0捕获输入，通道0  */

#define FUNC_PIO0_2         PIO0_2_GPIO

#define PIO0_2_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO0_2_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO0_2_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO0_2_REPEATER     0x03                                        /*  中继模式                    */

#define PIO0_2_MODE         PIO0_2_PULLUP

#define PIO0_2_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO0_2_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO0_2_HYS          PIO0_2_HYSDISABLE

#define PIO0_2_CON         (FUNC_PIO0_2 | (PIO0_2_MODE << 3) | (PIO0_2_HYS << 5))
/*********************************************************************************************************
  P0_3口引脚功能配置
*********************************************************************************************************/
#define PIO0_3_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO0_3         PIO0_3_GPIO

#define PIO0_3_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO0_3_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO0_3_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO0_3_REPEATER     0x03                                        /*  中继模式                    */

#define PIO0_3_MODE         PIO0_3_PULLUP

#define PIO0_3_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO0_3_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO0_3_HYS          PIO0_3_HYSDISABLE

#define PIO0_3_CON         (FUNC_PIO0_3 | (PIO0_3_MODE << 3) | (PIO0_3_HYS << 5))
/*********************************************************************************************************
  P0_4口引脚功能配置
*********************************************************************************************************/
#define PIO0_4_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_4_SCL          0x01                                        /*  I2C时钟线（开漏）           */

#define FUNC_PIO0_4         PIO0_4_GPIO

#define PIO0_4_STD_I2C      0x00                                        /*  标准/快速I2C模式            */
#define PIO0_4_STD_IO       0x01                                        /*  标准IO模式                  */
#define PIO0_4_FM_I2C       0x02                                        /*  快速I2C模式                 */

#define PIO0_4_MODE         PIO0_4_STD_I2C

#define PIO0_4_CON         (FUNC_PIO0_4 | (PIO0_4_MODE << 8))
/*********************************************************************************************************
  P0_5口引脚功能配置
*********************************************************************************************************/
#define PIO0_5_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_5_SDA          0x01                                        /*  I2C数据线（开漏）           */

#define FUNC_PIO0_5         PIO0_5_GPIO

#define PIO0_5_STD_I2C      0x00                                        /*  标准/快速I2C模式            */
#define PIO0_5_STD_IO       0x01                                        /*  标准IO模式                  */
#define PIO0_5_FM_I2C       0x02                                        /*  快速I2C模式                 */

#define PIO0_5_MODE         PIO0_5_STD_I2C

#define PIO0_5_CON         (FUNC_PIO0_5 | (PIO0_5_MODE << 8))
/*********************************************************************************************************
  P0_6口引脚功能配置
*********************************************************************************************************/
#define PIO0_6_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_6_SCK0         0x02                                        /*  SSP时钟线                   */

#define FUNC_PIO0_6         PIO0_6_GPIO

#define PIO0_6_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO0_6_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO0_6_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO0_6_REPEATER     0x03                                        /*  中继模式                    */

#define PIO0_6_MODE         PIO0_6_PULLUP

#define PIO0_6_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO0_6_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO0_6_HYS          PIO0_6_HYSDISABLE

#define PIO0_6_CON         (FUNC_PIO0_6 | (PIO0_6_MODE << 3) | (PIO0_6_HYS << 5))
/*********************************************************************************************************
  P0_7口引脚功能配置
*********************************************************************************************************/
#define PIO0_7_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_7_CTS          0x01                                        /*  UART清零发送信号CTS         */

#define FUNC_PIO0_7         PIO0_7_GPIO

#define PIO0_7_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO0_7_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO0_7_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO0_7_REPEATER     0x03                                        /*  中继模式                    */

#define PIO0_7_MODE         PIO0_7_PULLUP

#define PIO0_7_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO0_7_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO0_7_HYS          PIO0_7_HYSDISABLE

#define PIO0_7_CON         (FUNC_PIO0_7 | (PIO0_7_MODE << 3) | (PIO0_7_HYS << 5))
/*********************************************************************************************************
  P0_8口引脚功能配置
*********************************************************************************************************/
#define PIO0_8_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_8_MISO0        0x01                                        /*  SSP主输入从输出数据线       */
#define PIO0_8_CT16B0_MAT0  0x02                                        /*  16位定时器0匹配输出，通道0  */

#define FUNC_PIO0_8         PIO0_8_GPIO

#define PIO0_8_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO0_8_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO0_8_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO0_8_REPEATER     0x03                                        /*  中继模式                    */

#define PIO0_8_MODE         PIO0_8_PULLUP

#define PIO0_8_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO0_8_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO0_8_HYS          PIO0_8_HYSDISABLE

#define PIO0_8_CON         (FUNC_PIO0_8 | (PIO0_8_MODE << 3) | (PIO0_8_HYS << 5))
/*********************************************************************************************************
  P0_9口引脚功能配置
*********************************************************************************************************/
#define PIO0_9_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_9_MOSI0        0x01                                        /*  SSP主输出从输入数据线       */
#define PIO0_9_CT16B0_MAT1  0x02                                        /*  16位定时器0匹配输出，通道1  */

#define FUNC_PIO0_9         PIO0_9_GPIO

#define PIO0_9_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO0_9_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO0_9_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO0_9_REPEATER     0x03                                        /*  中继模式                    */

#define PIO0_9_MODE         PIO0_9_PULLUP

#define PIO0_9_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO0_9_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO0_9_HYS          PIO0_9_HYSDISABLE

#define PIO0_9_CON         (FUNC_PIO0_9 | (PIO0_9_MODE << 3) | (PIO0_9_HYS << 5))
/*********************************************************************************************************
  P0_10口引脚功能配置
*********************************************************************************************************/
#define PIO0_10_SWCLK       0x00                                        /*  串行调试时钟线              */
#define PIO0_10_GPIO        0x01                                        /*  GPIO                        */
#define PIO0_10_SCK0        0x02                                        /*  SSP时钟线                   */
#define PIO0_10_CT16B0_MAT2 0x03                                        /*  16位定时器0匹配输出，通道2  */

#define FUNC_PIO0_10        PIO0_10_SWCLK

#define PIO0_10_INACTIVE    0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO0_10_PULLDOWN    0x01                                        /*  内部下拉                    */
#define PIO0_10_PULLUP      0x02                                        /*  内部上拉                    */
#define PIO0_10_REPEATER    0x03                                        /*  中继模式                    */

#define PIO0_10_MODE        PIO0_10_PULLUP

#define PIO0_10_HYSDISABLE  0                                           /*  滞后禁能                    */
#define PIO0_10_HYSENABLE   1                                           /*  滞后使能                    */

#define PIO0_10_HYS         PIO0_10_HYSDISABLE

#define PIO0_10_CON        (FUNC_PIO0_10 | (PIO0_10_MODE << 3) | (PIO0_10_HYS << 5))
/*********************************************************************************************************
  P0_11口引脚功能配置
*********************************************************************************************************/
#define PIO0_11_TDI         0x00                                        /*  JTAG数据输入                */
#define PIO0_11_GPIO        0x01                                        /*  GPIO                        */
#define PIO0_11_AD0         0x02                                        /*  AD转换通道0                 */
#define PIO0_11_CT32B0_MAT3 0x03                                        /*  32位定时器0匹配输出，通道3  */

#define FUNC_PIO0_11        PIO0_11_TDI

#define PIO0_11_INACTIVE    0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO0_11_PULLDOWN    0x01                                        /*  内部下拉                    */
#define PIO0_11_PULLUP      0x02                                        /*  内部上拉                    */
#define PIO0_11_REPEATER    0x03                                        /*  中继模式                    */

#define PIO0_11_MODE        PIO0_11_PULLUP

#define PIO0_11_HYSDISABLE  0                                           /*  滞后禁能                    */
#define PIO0_11_HYSENABLE   1                                           /*  滞后使能                    */

#define PIO0_11_HYS         PIO0_11_HYSDISABLE

#define PIO0_11_ANALOG      0                                           /*  模拟输入模式                */
#define PIO0_11_DIGITAL     1                                           /*  数字模式                    */

#define PIO0_11_ADMODE      PIO0_11_DIGITAL

#define PIO0_11_CON        (FUNC_PIO0_11 | (PIO0_11_MODE << 3) | \
                           (PIO0_11_HYS << 5) | (PIO0_11_ADMODE << 7))
/*********************************************************************************************************
  P1_0口引脚功能配置
*********************************************************************************************************/
#define PIO1_0_TMS          0x00                                        /*  JTAG模式选择                */
#define PIO1_0_GPIO         0x01                                        /*  GPIO                        */
#define PIO1_0_AD1          0x02                                        /*  AD转换通道1                 */
#define PIO1_0_CT32B1_CAP0  0x03                                        /*  32位定时器1捕获输入，通道0  */

#define FUNC_PIO1_0         PIO1_0_TMS

#define PIO1_0_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_0_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO1_0_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO1_0_REPEATER     0x03                                        /*  中继模式                    */

#define PIO1_0_MODE         PIO1_0_PULLUP

#define PIO1_0_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO1_0_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO1_0_HYS          PIO1_0_HYSDISABLE
 
#define PIO1_0_ANALOG       0                                           /*  模拟输入模式                */
#define PIO1_0_DIGITAL      1                                           /*  数字模式                    */

#define PIO1_0_ADMODE       PIO1_0_DIGITAL

#define PIO1_0_CON         (FUNC_PIO1_0 | (PIO1_0_MODE << 3) | \
                           (PIO1_0_HYS << 5) | (PIO1_0_ADMODE << 7))
/*********************************************************************************************************
  P1_1口引脚功能配置
*********************************************************************************************************/
#define PIO1_1_TDO          0x00                                        /*  JTAG数据输出                */
#define PIO1_1_GPIO         0x01                                        /*  GPIO                        */
#define PIO1_1_AD2          0x02                                        /*  AD转换通道2                 */
#define PIO1_1_CT32B1_MAT0  0x03                                        /*  32位定时器1匹配输出，通道0  */

#define FUNC_PIO1_1         PIO1_1_TDO

#define PIO1_1_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_1_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO1_1_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO1_1_REPEATER     0x03                                        /*  中继模式                    */

#define PIO1_1_MODE         PIO1_1_PULLUP

#define PIO1_1_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO1_1_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO1_1_HYS          PIO1_1_HYSDISABLE

#define PIO1_1_ANALOG       0                                           /*  模拟输入模式                */
#define PIO1_1_DIGITAL      1                                           /*  数字模式                    */

#define PIO1_1_ADMODE       PIO1_1_DIGITAL

#define PIO1_1_CON         (FUNC_PIO1_1 | (PIO1_1_MODE << 3) | \
                           (PIO1_1_HYS << 5) | (PIO1_1_ADMODE << 7))
/*********************************************************************************************************
  P1_2口引脚功能配置
*********************************************************************************************************/
#define PIO1_2_TRST         0x00                                        /*  JTAG复位                    */
#define PIO1_2_GPIO         0x01                                        /*  GPIO                        */
#define PIO1_2_AD3          0x02                                        /*  AD转换通道3                 */
#define PIO1_2_CT32B1_MAT1  0x03                                        /*  32位定时器1匹配输出，通道1  */

#define FUNC_PIO1_2         PIO1_2_TRST

#define PIO1_2_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_2_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO1_2_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO1_2_REPEATER     0x03                                        /*  中继模式                    */

#define PIO1_2_MODE         PIO1_2_PULLUP

#define PIO1_2_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO1_2_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO1_2_HYS          PIO1_2_HYSDISABLE

#define PIO1_2_ANALOG       0                                           /*  模拟输入模式                */
#define PIO1_2_DIGITAL      1                                           /*  数字模式                    */

#define PIO1_2_ADMODE       PIO1_2_DIGITAL

#define PIO1_2_CON         (FUNC_PIO1_2 | (PIO1_2_MODE << 3) | \
                           (PIO1_2_HYS << 5) | (PIO1_2_ADMODE << 7))
/*********************************************************************************************************
  P1_3口引脚功能配置
*********************************************************************************************************/
#define PIO1_3_SWDIO        0x00                                        /*  串行调试数据输入输出        */
#define PIO1_3_GPIO         0x01                                        /*  GPIO                        */
#define PIO1_3_AD4          0x02                                        /*  AD转换通道4                 */
#define PIO1_3_CT32B1_MAT2  0x03                                        /*  32位定时器1匹配输出，通道2  */

#define FUNC_PIO1_3         PIO1_3_SWDIO

#define PIO1_3_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_3_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO1_3_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO1_3_REPEATER     0x03                                        /*  中继模式                    */

#define PIO1_3_MODE         PIO1_3_PULLUP

#define PIO1_3_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO1_3_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO1_3_HYS          PIO1_3_HYSDISABLE

#define PIO1_3_ANALOG       0                                           /*  模拟输入模式                */
#define PIO1_3_DIGITAL      1                                           /*  数字模式                    */

#define PIO1_3_ADMODE       PIO1_3_DIGITAL

#define PIO1_3_CON         (FUNC_PIO1_3 | (PIO1_3_MODE << 3) | \
                           (PIO1_3_HYS << 5) | (PIO1_3_ADMODE << 7))
/*********************************************************************************************************
  P1_4口引脚功能配置
*********************************************************************************************************/
#define PIO1_4_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_4_AD5          0x01                                        /*  AD转换通道5                 */
#define PIO1_4_CT32B1_MAT3  0x02                                        /*  32位定时器1匹配输出，通道3  */

#define FUNC_PIO1_4         PIO1_4_GPIO

#define PIO1_4_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_4_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO1_4_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO1_4_REPEATER     0x03                                        /*  中继模式                    */

#define PIO1_4_MODE         PIO1_4_PULLUP

#define PIO1_4_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO1_4_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO1_4_HYS          PIO1_4_HYSDISABLE

#define PIO1_4_ANALOG       0                                           /*  模拟输入模式                */
#define PIO1_4_DIGITAL      1                                           /*  数字模式                    */

#define PIO1_4_ADMODE       PIO1_4_DIGITAL

#define PIO1_4_CON         (FUNC_PIO1_4 | (PIO1_4_MODE << 3) | \
                           (PIO1_4_HYS << 5) | (PIO1_4_ADMODE << 7))
/*********************************************************************************************************
  P1_5口引脚功能配置
*********************************************************************************************************/
#define PIO1_5_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_5_RTS          0x01                                        /*  UART请求发送引脚            */
#define PIO1_5_CT32B0_CAP0  0x02                                        /*  32位定时器0捕获输入，通道0  */

#define FUNC_PIO1_5         PIO1_5_GPIO

#define PIO1_5_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_5_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO1_5_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO1_5_REPEATER     0x03                                        /*  中继模式                    */

#define PIO1_5_MODE         PIO1_5_PULLUP

#define PIO1_5_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO1_5_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO1_5_HYS          PIO1_5_HYSDISABLE

#define PIO1_5_CON         (FUNC_PIO1_5 | (PIO1_5_MODE << 3) | (PIO1_5_HYS << 5))
/*********************************************************************************************************
  P1_6口引脚功能配置
*********************************************************************************************************/
#define PIO1_6_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_6_UART_RXD     0x01                                        /*  UART数据接收引脚            */
#define PIO1_6_CT32B0_MAT0  0x02                                        /*  32位定时器0匹配输出，通道0  */

#define FUNC_PIO1_6         PIO1_6_GPIO

#define PIO1_6_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_6_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO1_6_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO1_6_REPEATER     0x03                                        /*  中继模式                    */

#define PIO1_6_MODE         PIO1_6_PULLUP

#define PIO1_6_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO1_6_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO1_6_HYS          PIO1_6_HYSDISABLE

#define PIO1_6_CON         (FUNC_PIO1_6 | (PIO1_6_MODE << 3) | (PIO1_6_HYS << 5))
/*********************************************************************************************************
  P1_7口引脚功能配置
*********************************************************************************************************/
#define PIO1_7_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_7_UART_TXD     0x01                                        /*  UART数据发送引脚            */
#define PIO1_7_CT32B0_MAT1  0x02                                        /*  32位定时器0匹配输出，通道1  */

#define FUNC_PIO1_7         PIO1_7_GPIO
 
#define PIO1_7_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_7_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO1_7_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO1_7_REPEATER     0x03                                        /*  中继模式                    */

#define PIO1_7_MODE         PIO1_7_PULLUP

#define PIO1_7_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO1_7_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO1_7_HYS          PIO1_7_HYSDISABLE

#define PIO1_7_CON         (FUNC_PIO1_7 | (PIO1_7_MODE << 3) | (PIO1_7_HYS << 5))
/*********************************************************************************************************
  P1_8口引脚功能配置
*********************************************************************************************************/
#define PIO1_8_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_8_CT16B1_CAP0  0x01                                        /*  16位定时器1捕获输入，通道0  */

#define FUNC_PIO1_8         PIO1_8_GPIO

#define PIO1_8_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_8_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO1_8_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO1_8_REPEATER     0x03                                        /*  中继模式                    */

#define PIO1_8_MODE         PIO1_8_PULLUP

#define PIO1_8_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO1_8_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO1_8_HYS          PIO1_8_HYSDISABLE

#define PIO1_8_CON         (FUNC_PIO1_8 | (PIO1_8_MODE << 3) | (PIO1_8_HYS << 5))
/*********************************************************************************************************
  P1_9口引脚功能配置
*********************************************************************************************************/
#define PIO1_9_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_9_CT16B1_MAT0  0x01                                        /*  16位定时器1匹配输出，通道0  */

#define FUNC_PIO1_9         PIO1_9_GPIO

#define PIO1_9_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_9_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO1_9_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO1_9_REPEATER     0x03                                        /*  中继模式                    */

#define PIO1_9_MODE         PIO1_9_PULLUP

#define PIO1_9_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO1_9_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO1_9_HYS          PIO1_9_HYSDISABLE

#define PIO1_9_CON          (FUNC_PIO1_9 | (PIO1_9_MODE << 3) | (PIO1_9_HYS << 5))
/*********************************************************************************************************
  P1_10口引脚功能配置
*********************************************************************************************************/
#define PIO1_10_GPIO        0x00                                        /*  GPIO                        */
#define PIO1_10_AD6         0x01                                        /*  AD转换通道6                 */
#define PIO1_10_CT16B1_MAT1 0x02                                        /*  16位定时器1匹配输出，通道1  */

#define FUNC_PIO1_10        PIO1_10_GPIO

#define PIO1_10_INACTIVE    0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_10_PULLDOWN    0x01                                        /*  内部下拉                    */
#define PIO1_10_PULLUP      0x02                                        /*  内部上拉                    */
#define PIO1_10_REPEATER    0x03                                        /*  中继模式                    */

#define PIO1_10_MODE        PIO1_10_PULLUP

#define PIO1_10_HYSDISABLE  0                                           /*  滞后禁能                    */
#define PIO1_10_HYSENABLE   1                                           /*  滞后使能                    */

#define PIO1_10_HYS         PIO1_10_HYSDISABLE

#define PIO1_10_ANALOG      0                                           /*  模拟输入模式                */
#define PIO1_10_DIGITAL     1                                           /*  数字模式                    */

#define PIO1_10_ADMODE      PIO1_10_DIGITAL

#define PIO1_10_CON         (FUNC_PIO1_10 | (PIO1_10_MODE << 3) | \
                            (PIO1_10_HYS << 5) | (PIO1_10_ADMODE << 7))
/*********************************************************************************************************
  P1_11口引脚功能配置
*********************************************************************************************************/
#define PIO1_11_GPIO        0x00                                        /*  GPIO                        */
#define PIO1_11_AD7         0x01                                        /*  AD转换通道7                 */

#define FUNC_PIO1_11        PIO1_11_GPIO

#define PIO1_11_INACTIVE    0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO1_11_PULLDOWN    0x01                                        /*  内部下拉                    */
#define PIO1_11_PULLUP      0x02                                        /*  内部上拉                    */
#define PIO1_11_REPEATER    0x03                                        /*  中继模式                    */

#define PIO1_11_MODE        PIO1_0_PULLUP

#define PIO1_11_HYSDISABLE  0                                           /*  滞后禁能                    */
#define PIO1_11_HYSENABLE   1                                           /*  滞后使能                    */

#define PIO1_11_HYS         PIO1_11_HYSDISABLE

#define PIO1_11_ANALOG      0                                           /*  模拟输入模式                */
#define PIO1_11_DIGITAL     1                                           /*  数字模式                    */

#define PIO1_11_ADMODE      PIO1_11_DIGITAL

#define PIO1_11_CON        (FUNC_PIO1_11 | (PIO1_11_MODE << 3) | \
                           (PIO1_11_HYS << 5) | (PIO1_11_ADMODE << 7))
/*********************************************************************************************************
  P2_0口引脚功能配置
*********************************************************************************************************/
#define PIO2_0_GPIO         0x00                                        /*  GPIO                        */
#define PIO2_0_DTR          0x01                                        /*  UART终端就绪DTR             */
#define PIO2_0_SSEL1        0x02                                        /*  SSP总线从机选择             */

#define FUNC_PIO2_0         PIO2_0_GPIO

#define PIO2_0_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO2_0_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO2_0_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO2_0_REPEATER     0x03                                        /*  中继模式                    */

#define PIO2_0_MODE         PIO2_0_PULLUP

#define PIO2_0_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO2_0_HYSENABLE    1                                           /*  滞后使能                    */
 
#define PIO2_0_HYS          PIO2_0_HYSDISABLE

#define PIO2_0_CON         (FUNC_PIO2_0 | (PIO2_0_MODE << 3) | (PIO2_0_HYS << 5))
/*********************************************************************************************************
  P2_1口引脚功能配置
  注意： 仅LPC1113的LQFP48封装;LPC1114的LQFP48封装和PLCC44封装具有此引脚
*********************************************************************************************************/
#define PIO2_1_GPIO         0x00                                        /*  GPIO                        */
#define PIO2_1_DSR          0x01                                        /*  UART数据设置就绪DSR         */
#define PIO2_1_SCK1         0x02                                        /*  SSP总线时钟线               */

#define FUNC_PIO2_1         PIO2_1_GPIO

#define PIO2_1_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO2_1_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO2_1_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO2_1_REPEATER     0x03                                        /*  中继模式                    */

#define PIO2_1_MODE         PIO2_1_PULLUP

#define PIO2_1_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO2_1_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO2_1_HYS          PIO2_1_HYSDISABLE

#define PIO2_1_CON         (FUNC_PIO2_1 | (PIO2_1_MODE << 3) | (PIO2_1_HYS << 5))
/*********************************************************************************************************
  P2_2口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装和PLCC44封装具有此引脚
*********************************************************************************************************/
#define PIO2_2_GPIO         0x00                                        /*  GPIO                        */
#define PIO2_2_DCD          0x01                                        /*  UART数据载波检测输入DCD     */
#define PIO2_2_MISO1        0x02                                        /*  SSP主输入从输出数据线       */

#define FUNC_PIO2_2         PIO2_2_GPIO

#define PIO2_2_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO2_2_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO2_2_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO2_2_REPEATER     0x03                                        /*  中继模式                    */

#define PIO2_2_MODE         PIO2_2_PULLUP

#define PIO2_2_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO2_2_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO2_2_HYS          PIO2_2_HYSDISABLE

#define PIO2_2_CON         (FUNC_PIO2_2 | (PIO2_2_MODE << 3) | (PIO2_2_HYS << 5))
/*********************************************************************************************************
  P2_3口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装和PLCC44封装具有此引脚
*********************************************************************************************************/
#define PIO2_3_GPIO         0x00                                        /*  GPIO                        */
#define PIO2_3_RI           0x01                                        /*  UART振铃信号RI              */
#define PIO2_3_MOSI1        0x02                                        /*  SSP主输出从输入数据线       */

#define FUNC_PIO2_3         PIO2_3_GPIO

#define PIO2_3_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO2_3_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO2_3_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO2_3_REPEATER     0x03                                        /*  中继模式                    */

#define PIO2_3_MODE         PIO2_3_PULLUP

#define PIO2_3_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO2_3_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO2_3_HYS          PIO2_3_HYSDISABLE

#define PIO2_3_CON         (FUNC_PIO2_3 | (PIO2_3_MODE << 3) | (PIO2_3_HYS << 5))
/*********************************************************************************************************
  P2_4口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装和PLCC44封装具有此引脚
*********************************************************************************************************/
#define PIO2_4_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_4         PIO2_4_GPIO

#define PIO2_4_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO2_4_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO2_4_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO2_4_REPEATER     0x03                                        /*  中继模式                    */

#define PIO2_4_MODE         PIO2_4_PULLUP

#define PIO2_4_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO2_4_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO2_4_HYS          PIO2_4_HYSDISABLE

#define PIO2_4_CON         (FUNC_PIO2_4 | (PIO2_4_MODE << 3) | (PIO2_4_HYS << 5))
/*********************************************************************************************************
  P2_5口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装和PLCC44封装具有此引脚
*********************************************************************************************************/
#define PIO2_5_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_5         PIO2_5_GPIO

#define PIO2_5_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO2_5_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO2_5_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO2_5_REPEATER     0x03                                        /*  中继模式                    */

#define PIO2_5_MODE         PIO2_5_PULLUP

#define PIO2_5_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO2_5_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO2_5_HYS          PIO2_5_HYSDISABLE

#define PIO2_5_CON         (FUNC_PIO2_5 | (PIO2_5_MODE << 3) | (PIO2_5_HYS << 5))
/*********************************************************************************************************
  P2_6口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装和PLCC44封装具有此引脚
*********************************************************************************************************/
#define PIO2_6_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_6         PIO2_6_GPIO

#define PIO2_6_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO2_6_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO2_6_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO2_6_REPEATER     0x03                                        /*  中继模式                    */

#define PIO2_6_MODE         PIO2_6_PULLUP

#define PIO2_6_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO2_6_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO2_6_HYS          PIO2_6_HYSDISABLE

#define PIO2_6_CON         (FUNC_PIO2_6 | (PIO2_6_MODE << 3) | (PIO2_6_HYS << 5))
/*********************************************************************************************************
  P2_7口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装和PLCC44封装具有此引脚
*********************************************************************************************************/
#define PIO2_7_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_7         PIO2_7_GPIO

#define PIO2_7_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO2_7_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO2_7_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO2_7_REPEATER     0x03                                        /*  中继模式                    */

#define PIO2_7_MODE         PIO2_7_PULLUP

#define PIO2_7_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO2_7_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO2_7_HYS          PIO2_7_HYSDISABLE

#define PIO2_7_CON         (FUNC_PIO2_7 | (PIO2_7_MODE << 3) | (PIO2_7_HYS << 5))
/*********************************************************************************************************
  P2_8口引脚功能配置
  仅LPC1113的LQFP48封装;LPC1114的LQFP48封装和PLCC44封装具有此引脚
*********************************************************************************************************/
#define PIO2_8_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_8         PIO2_8_GPIO

#define PIO2_8_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO2_8_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO2_8_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO2_8_REPEATER     0x03                                        /*  中继模式                    */

#define PIO2_8_MODE         PIO2_8_PULLUP

#define PIO2_8_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO2_8_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO2_8_HYS          PIO2_8_HYSDISABLE

#define PIO2_8_CON         (FUNC_PIO2_8 | (PIO2_8_MODE << 3) | (PIO2_8_HYS << 5))
/*********************************************************************************************************
  P2_9口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装和PLCC44封装具有此引脚
*********************************************************************************************************/
#define PIO2_9_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_9         PIO2_9_GPIO

#define PIO2_9_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO2_9_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO2_9_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO2_9_REPEATER     0x03                                        /*  中继模式                    */

#define PIO2_9_MODE         PIO2_9_PULLUP

#define PIO2_9_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO2_9_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO2_9_HYS          PIO2_9_HYSDISABLE

#define PIO2_9_CON         (FUNC_PIO2_9 | (PIO2_9_MODE << 3) | (PIO2_9_HYS << 5))
/*********************************************************************************************************
  P2_10口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装和PLCC44封装具有此引脚
*********************************************************************************************************/
#define PIO2_10_GPIO        0x00                                        /*  GPIO                        */

#define FUNC_PIO2_10        PIO2_10_GPIO

#define PIO2_10_INACTIVE    0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO2_10_PULLDOWN    0x01                                        /*  内部下拉                    */
#define PIO2_10_PULLUP      0x02                                        /*  内部上拉                    */
#define PIO2_10_REPEATER    0x03                                        /*  中继模式                    */

#define PIO2_10_MODE        PIO2_10_PULLUP

#define PIO2_10_HYSDISABLE  0                                           /*  滞后禁能                    */
#define PIO2_10_HYSENABLE   1                                           /*  滞后使能                    */

#define PIO2_10_HYS         PIO2_10_HYSDISABLE

#define PIO2_10_CON        (FUNC_PIO2_10 | (PIO2_10_MODE << 3) | (PIO2_10_HYS << 5))
/*********************************************************************************************************
  P2_11口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装和PLCC44封装具有此引脚
*********************************************************************************************************/
#define PIO2_11_GPIO        0x00                                        /*  GPIO                       */
#define PIO2_11_SCK0        0x01                                        /*  SSP的SCK引脚               */

#define FUNC_PIO2_11        PIO2_11_GPIO

#define PIO2_11_INACTIVE    0x00                                        /*  高阻（既不上拉也不下拉）   */
#define PIO2_11_PULLDOWN    0x01                                        /*  内部下拉                   */
#define PIO2_11_PULLUP      0x02                                        /*  内部上拉                   */
#define PIO2_11_REPEATER    0x03                                        /*  中继模式                   */

#define PIO2_11_MODE        PIO2_11_PULLUP

#define PIO2_11_HYSDISABLE  0                                           /*  滞后禁能                   */
#define PIO2_11_HYSENABLE   1                                           /*  滞后使能                   */

#define PIO2_11_HYS         PIO2_11_HYSDISABLE

#define PIO2_11_CON        (FUNC_PIO2_11 | (PIO2_11_MODE << 3) | (PIO2_11_HYS << 5))
/*********************************************************************************************************
  P3_0口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装具有此引脚
*********************************************************************************************************/
#define PIO3_0_GPIO         0x00                                        /*  GPIO                        */
#define PIO3_0_DTR          0x01                                        /*  UART终端就绪DTR             */

#define FUNC_PIO3_0         PIO3_0_GPIO

#define PIO3_0_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO3_0_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO3_0_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO3_0_REPEATER     0x03                                        /*  中继模式                    */
 
#define PIO3_0_MODE         PIO3_0_PULLUP

#define PIO3_0_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO3_0_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO3_0_HYS          PIO3_0_HYSDISABLE

#define PIO3_0_CON         (FUNC_PIO3_0 | (PIO3_0_MODE << 3) | (PIO3_0_HYS << 5))
/*********************************************************************************************************
  P3_1口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装具有此引脚
*********************************************************************************************************/
#define PIO3_1_GPIO         0x00                                        /*  GPIO                        */
#define PIO3_1_DSR          0x01                                        /*  UART数据设置就绪DSR         */

#define FUNC_PIO3_1         PIO3_1_GPIO

#define PIO3_1_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO3_1_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO3_1_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO3_1_REPEATER     0x03                                        /*  中继模式                    */

#define PIO3_1_MODE         PIO3_1_PULLUP

#define PIO3_1_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO3_1_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO3_1_HYS          PIO3_1_HYSDISABLE

#define PIO3_1_CON         (FUNC_PIO3_1 | (PIO3_1_MODE << 3) | (PIO3_1_HYS << 5))
/*********************************************************************************************************
  P3_2口引脚功能配置
  注意：仅LPC1111;LPC1112;LPC1113;LPC1114的HVQFN33封装和LQFP48封装具有此引脚
*********************************************************************************************************/
#define PIO3_2_GPIO         0x00                                        /*  GPIO                        */
#define PIO3_2_DCD          0x01                                        /*  UART数据载波检测输入DCD     */

#define FUNC_PIO3_2         PIO3_2_GPIO

#define PIO3_2_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO3_2_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO3_2_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO3_2_REPEATER     0x03                                        /*  中继模式                    */

#define PIO3_2_MODE         PIO3_2_PULLUP

#define PIO3_2_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO3_2_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO3_2_HYS          PIO3_2_HYSDISABLE

#define PIO3_2_CON         (FUNC_PIO3_2 | (PIO3_2_MODE << 3) | (PIO3_2_HYS << 5))
/*********************************************************************************************************
  P3_3口引脚功能配置
  注意：仅LPC1113的LQFP48封装;LPC1114的LQFP48封装具有此引脚
*********************************************************************************************************/
#define PIO3_3_GPIO         0x00                                        /*  GPIO                        */
#define PIO3_3_RI           0x01                                        /*  UART振铃信号RI              */

#define FUNC_PIO3_3         PIO3_3_GPIO

#define PIO3_3_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO3_3_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO3_3_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO3_3_REPEATER     0x03                                        /*  中继模式                    */

#define PIO3_3_MODE         PIO3_3_PULLUP

#define PIO3_3_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO3_3_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO3_3_HYS          PIO3_3_HYSDISABLE

#define PIO3_3_CON         (FUNC_PIO3_3 | (PIO3_3_MODE << 3) | (PIO3_3_HYS << 5))
/*********************************************************************************************************
  P3_4口引脚功能配置
*********************************************************************************************************/
#define PIO3_4_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO3_4         PIO3_4_GPIO

#define PIO3_4_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO3_4_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO3_4_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO3_4_REPEATER     0x03                                        /*  中继模式                    */

#define PIO3_4_MODE         PIO3_4_PULLUP

#define PIO3_4_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO3_4_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO3_4_HYS          PIO3_4_HYSDISABLE

#define PIO3_4_CON         (FUNC_PIO3_4 | (PIO3_4_MODE << 3) | (PIO3_4_HYS << 5))
/*********************************************************************************************************
  P3_5口引脚功能配置
*********************************************************************************************************/
#define PIO3_5_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO3_5         PIO3_5_GPIO

#define PIO3_5_INACTIVE     0x00                                        /*  高阻（既不上拉也不下拉）    */
#define PIO3_5_PULLDOWN     0x01                                        /*  内部下拉                    */
#define PIO3_5_PULLUP       0x02                                        /*  内部上拉                    */
#define PIO3_5_REPEATER     0x03                                        /*  中继模式                    */

#define PIO3_5_MODE         PIO3_5_PULLUP

#define PIO3_5_HYSDISABLE   0                                           /*  滞后禁能                    */
#define PIO3_5_HYSENABLE    1                                           /*  滞后使能                    */

#define PIO3_5_HYS          PIO3_5_HYSDISABLE

#define PIO3_5_CON         (FUNC_PIO3_5 | (PIO3_5_MODE << 3) | (PIO3_5_HYS << 5))

/*********************************************************************************************************
** Function name:       pinInit
** Descriptions:        初始化所有的引脚配置，完成引脚连接、上下拉电阻设置
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern void pinInit(void);

#endif                                                                  /*  __LPC1300PINCFG_H           */
/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
