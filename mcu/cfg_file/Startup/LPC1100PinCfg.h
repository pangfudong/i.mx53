/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           LPC1100PinCfg.h
** Last modified Date:  2009-10-22
** Last Version:        V1.01
** Descriptions:        LPC1300ϵ��CPU����������ģʽ����
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
  IO�����Ź������ú궨��
*********************************************************************************************************/

/*********************************************************************************************************
  P0_0�����Ź�������
*********************************************************************************************************/
#define PIO0_0_RESET        0x00                                        /*  ��λ                        */
#define PIO0_0_GPIO         0x01                                        /*  GPIO                        */

#define FUNC_PIO0_0         PIO0_0_RESET

#define PIO0_0_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO0_0_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO0_0_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO0_0_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO0_0_MODE         PIO0_0_PULLUP

#define PIO0_0_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO0_0_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO0_0_HYS          PIO0_0_HYSDISABLE

#define PIO0_0_CON         (FUNC_PIO0_0 | (PIO0_0_MODE << 3) | (PIO0_0_HYS << 5))
/*********************************************************************************************************
  P0_1�����Ź�������
*********************************************************************************************************/
#define PIO0_1_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_1_CLKOUT       0x01                                        /*  ʱ�����                    */
#define PIO0_1_CT32B0_MAT2  0x02                                        /*  32λ��ʱ��0ƥ�������ͨ��2  */

#define FUNC_PIO0_1         PIO0_1_GPIO

#define PIO0_1_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO0_1_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO0_1_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO0_1_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO0_1_MODE         PIO0_1_PULLUP

#define PIO0_1_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO0_1_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO0_1_HYS          PIO0_1_HYSDISABLE

#define PIO0_1_CON         (FUNC_PIO0_1 | (PIO0_1_MODE << 3) | (PIO0_1_HYS << 5))
/*********************************************************************************************************
  P0_2�����Ź�������
*********************************************************************************************************/
#define PIO0_2_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_2_SSEL0        0x01                                        /*  SSP���ߴӻ�ѡ��             */
#define PIO0_2_CT16B0_CAP0  0x02                                        /*  16λ��ʱ��0�������룬ͨ��0  */

#define FUNC_PIO0_2         PIO0_2_GPIO

#define PIO0_2_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO0_2_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO0_2_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO0_2_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO0_2_MODE         PIO0_2_PULLUP

#define PIO0_2_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO0_2_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO0_2_HYS          PIO0_2_HYSDISABLE

#define PIO0_2_CON         (FUNC_PIO0_2 | (PIO0_2_MODE << 3) | (PIO0_2_HYS << 5))
/*********************************************************************************************************
  P0_3�����Ź�������
*********************************************************************************************************/
#define PIO0_3_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO0_3         PIO0_3_GPIO

#define PIO0_3_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO0_3_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO0_3_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO0_3_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO0_3_MODE         PIO0_3_PULLUP

#define PIO0_3_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO0_3_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO0_3_HYS          PIO0_3_HYSDISABLE

#define PIO0_3_CON         (FUNC_PIO0_3 | (PIO0_3_MODE << 3) | (PIO0_3_HYS << 5))
/*********************************************************************************************************
  P0_4�����Ź�������
*********************************************************************************************************/
#define PIO0_4_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_4_SCL          0x01                                        /*  I2Cʱ���ߣ���©��           */

#define FUNC_PIO0_4         PIO0_4_GPIO

#define PIO0_4_STD_I2C      0x00                                        /*  ��׼/����I2Cģʽ            */
#define PIO0_4_STD_IO       0x01                                        /*  ��׼IOģʽ                  */
#define PIO0_4_FM_I2C       0x02                                        /*  ����I2Cģʽ                 */

#define PIO0_4_MODE         PIO0_4_STD_I2C

#define PIO0_4_CON         (FUNC_PIO0_4 | (PIO0_4_MODE << 8))
/*********************************************************************************************************
  P0_5�����Ź�������
*********************************************************************************************************/
#define PIO0_5_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_5_SDA          0x01                                        /*  I2C�����ߣ���©��           */

#define FUNC_PIO0_5         PIO0_5_GPIO

#define PIO0_5_STD_I2C      0x00                                        /*  ��׼/����I2Cģʽ            */
#define PIO0_5_STD_IO       0x01                                        /*  ��׼IOģʽ                  */
#define PIO0_5_FM_I2C       0x02                                        /*  ����I2Cģʽ                 */

#define PIO0_5_MODE         PIO0_5_STD_I2C

#define PIO0_5_CON         (FUNC_PIO0_5 | (PIO0_5_MODE << 8))
/*********************************************************************************************************
  P0_6�����Ź�������
*********************************************************************************************************/
#define PIO0_6_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_6_SCK0         0x02                                        /*  SSPʱ����                   */

#define FUNC_PIO0_6         PIO0_6_GPIO

#define PIO0_6_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO0_6_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO0_6_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO0_6_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO0_6_MODE         PIO0_6_PULLUP

#define PIO0_6_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO0_6_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO0_6_HYS          PIO0_6_HYSDISABLE

#define PIO0_6_CON         (FUNC_PIO0_6 | (PIO0_6_MODE << 3) | (PIO0_6_HYS << 5))
/*********************************************************************************************************
  P0_7�����Ź�������
*********************************************************************************************************/
#define PIO0_7_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_7_CTS          0x01                                        /*  UART���㷢���ź�CTS         */

#define FUNC_PIO0_7         PIO0_7_GPIO

#define PIO0_7_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO0_7_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO0_7_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO0_7_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO0_7_MODE         PIO0_7_PULLUP

#define PIO0_7_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO0_7_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO0_7_HYS          PIO0_7_HYSDISABLE

#define PIO0_7_CON         (FUNC_PIO0_7 | (PIO0_7_MODE << 3) | (PIO0_7_HYS << 5))
/*********************************************************************************************************
  P0_8�����Ź�������
*********************************************************************************************************/
#define PIO0_8_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_8_MISO0        0x01                                        /*  SSP����������������       */
#define PIO0_8_CT16B0_MAT0  0x02                                        /*  16λ��ʱ��0ƥ�������ͨ��0  */

#define FUNC_PIO0_8         PIO0_8_GPIO

#define PIO0_8_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO0_8_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO0_8_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO0_8_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO0_8_MODE         PIO0_8_PULLUP

#define PIO0_8_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO0_8_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO0_8_HYS          PIO0_8_HYSDISABLE

#define PIO0_8_CON         (FUNC_PIO0_8 | (PIO0_8_MODE << 3) | (PIO0_8_HYS << 5))
/*********************************************************************************************************
  P0_9�����Ź�������
*********************************************************************************************************/
#define PIO0_9_GPIO         0x00                                        /*  GPIO                        */
#define PIO0_9_MOSI0        0x01                                        /*  SSP�����������������       */
#define PIO0_9_CT16B0_MAT1  0x02                                        /*  16λ��ʱ��0ƥ�������ͨ��1  */

#define FUNC_PIO0_9         PIO0_9_GPIO

#define PIO0_9_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO0_9_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO0_9_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO0_9_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO0_9_MODE         PIO0_9_PULLUP

#define PIO0_9_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO0_9_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO0_9_HYS          PIO0_9_HYSDISABLE

#define PIO0_9_CON         (FUNC_PIO0_9 | (PIO0_9_MODE << 3) | (PIO0_9_HYS << 5))
/*********************************************************************************************************
  P0_10�����Ź�������
*********************************************************************************************************/
#define PIO0_10_SWCLK       0x00                                        /*  ���е���ʱ����              */
#define PIO0_10_GPIO        0x01                                        /*  GPIO                        */
#define PIO0_10_SCK0        0x02                                        /*  SSPʱ����                   */
#define PIO0_10_CT16B0_MAT2 0x03                                        /*  16λ��ʱ��0ƥ�������ͨ��2  */

#define FUNC_PIO0_10        PIO0_10_SWCLK

#define PIO0_10_INACTIVE    0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO0_10_PULLDOWN    0x01                                        /*  �ڲ�����                    */
#define PIO0_10_PULLUP      0x02                                        /*  �ڲ�����                    */
#define PIO0_10_REPEATER    0x03                                        /*  �м�ģʽ                    */

#define PIO0_10_MODE        PIO0_10_PULLUP

#define PIO0_10_HYSDISABLE  0                                           /*  �ͺ����                    */
#define PIO0_10_HYSENABLE   1                                           /*  �ͺ�ʹ��                    */

#define PIO0_10_HYS         PIO0_10_HYSDISABLE

#define PIO0_10_CON        (FUNC_PIO0_10 | (PIO0_10_MODE << 3) | (PIO0_10_HYS << 5))
/*********************************************************************************************************
  P0_11�����Ź�������
*********************************************************************************************************/
#define PIO0_11_TDI         0x00                                        /*  JTAG��������                */
#define PIO0_11_GPIO        0x01                                        /*  GPIO                        */
#define PIO0_11_AD0         0x02                                        /*  ADת��ͨ��0                 */
#define PIO0_11_CT32B0_MAT3 0x03                                        /*  32λ��ʱ��0ƥ�������ͨ��3  */

#define FUNC_PIO0_11        PIO0_11_TDI

#define PIO0_11_INACTIVE    0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO0_11_PULLDOWN    0x01                                        /*  �ڲ�����                    */
#define PIO0_11_PULLUP      0x02                                        /*  �ڲ�����                    */
#define PIO0_11_REPEATER    0x03                                        /*  �м�ģʽ                    */

#define PIO0_11_MODE        PIO0_11_PULLUP

#define PIO0_11_HYSDISABLE  0                                           /*  �ͺ����                    */
#define PIO0_11_HYSENABLE   1                                           /*  �ͺ�ʹ��                    */

#define PIO0_11_HYS         PIO0_11_HYSDISABLE

#define PIO0_11_ANALOG      0                                           /*  ģ������ģʽ                */
#define PIO0_11_DIGITAL     1                                           /*  ����ģʽ                    */

#define PIO0_11_ADMODE      PIO0_11_DIGITAL

#define PIO0_11_CON        (FUNC_PIO0_11 | (PIO0_11_MODE << 3) | \
                           (PIO0_11_HYS << 5) | (PIO0_11_ADMODE << 7))
/*********************************************************************************************************
  P1_0�����Ź�������
*********************************************************************************************************/
#define PIO1_0_TMS          0x00                                        /*  JTAGģʽѡ��                */
#define PIO1_0_GPIO         0x01                                        /*  GPIO                        */
#define PIO1_0_AD1          0x02                                        /*  ADת��ͨ��1                 */
#define PIO1_0_CT32B1_CAP0  0x03                                        /*  32λ��ʱ��1�������룬ͨ��0  */

#define FUNC_PIO1_0         PIO1_0_TMS

#define PIO1_0_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_0_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO1_0_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO1_0_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO1_0_MODE         PIO1_0_PULLUP

#define PIO1_0_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO1_0_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO1_0_HYS          PIO1_0_HYSDISABLE
 
#define PIO1_0_ANALOG       0                                           /*  ģ������ģʽ                */
#define PIO1_0_DIGITAL      1                                           /*  ����ģʽ                    */

#define PIO1_0_ADMODE       PIO1_0_DIGITAL

#define PIO1_0_CON         (FUNC_PIO1_0 | (PIO1_0_MODE << 3) | \
                           (PIO1_0_HYS << 5) | (PIO1_0_ADMODE << 7))
/*********************************************************************************************************
  P1_1�����Ź�������
*********************************************************************************************************/
#define PIO1_1_TDO          0x00                                        /*  JTAG�������                */
#define PIO1_1_GPIO         0x01                                        /*  GPIO                        */
#define PIO1_1_AD2          0x02                                        /*  ADת��ͨ��2                 */
#define PIO1_1_CT32B1_MAT0  0x03                                        /*  32λ��ʱ��1ƥ�������ͨ��0  */

#define FUNC_PIO1_1         PIO1_1_TDO

#define PIO1_1_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_1_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO1_1_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO1_1_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO1_1_MODE         PIO1_1_PULLUP

#define PIO1_1_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO1_1_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO1_1_HYS          PIO1_1_HYSDISABLE

#define PIO1_1_ANALOG       0                                           /*  ģ������ģʽ                */
#define PIO1_1_DIGITAL      1                                           /*  ����ģʽ                    */

#define PIO1_1_ADMODE       PIO1_1_DIGITAL

#define PIO1_1_CON         (FUNC_PIO1_1 | (PIO1_1_MODE << 3) | \
                           (PIO1_1_HYS << 5) | (PIO1_1_ADMODE << 7))
/*********************************************************************************************************
  P1_2�����Ź�������
*********************************************************************************************************/
#define PIO1_2_TRST         0x00                                        /*  JTAG��λ                    */
#define PIO1_2_GPIO         0x01                                        /*  GPIO                        */
#define PIO1_2_AD3          0x02                                        /*  ADת��ͨ��3                 */
#define PIO1_2_CT32B1_MAT1  0x03                                        /*  32λ��ʱ��1ƥ�������ͨ��1  */

#define FUNC_PIO1_2         PIO1_2_TRST

#define PIO1_2_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_2_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO1_2_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO1_2_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO1_2_MODE         PIO1_2_PULLUP

#define PIO1_2_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO1_2_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO1_2_HYS          PIO1_2_HYSDISABLE

#define PIO1_2_ANALOG       0                                           /*  ģ������ģʽ                */
#define PIO1_2_DIGITAL      1                                           /*  ����ģʽ                    */

#define PIO1_2_ADMODE       PIO1_2_DIGITAL

#define PIO1_2_CON         (FUNC_PIO1_2 | (PIO1_2_MODE << 3) | \
                           (PIO1_2_HYS << 5) | (PIO1_2_ADMODE << 7))
/*********************************************************************************************************
  P1_3�����Ź�������
*********************************************************************************************************/
#define PIO1_3_SWDIO        0x00                                        /*  ���е��������������        */
#define PIO1_3_GPIO         0x01                                        /*  GPIO                        */
#define PIO1_3_AD4          0x02                                        /*  ADת��ͨ��4                 */
#define PIO1_3_CT32B1_MAT2  0x03                                        /*  32λ��ʱ��1ƥ�������ͨ��2  */

#define FUNC_PIO1_3         PIO1_3_SWDIO

#define PIO1_3_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_3_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO1_3_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO1_3_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO1_3_MODE         PIO1_3_PULLUP

#define PIO1_3_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO1_3_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO1_3_HYS          PIO1_3_HYSDISABLE

#define PIO1_3_ANALOG       0                                           /*  ģ������ģʽ                */
#define PIO1_3_DIGITAL      1                                           /*  ����ģʽ                    */

#define PIO1_3_ADMODE       PIO1_3_DIGITAL

#define PIO1_3_CON         (FUNC_PIO1_3 | (PIO1_3_MODE << 3) | \
                           (PIO1_3_HYS << 5) | (PIO1_3_ADMODE << 7))
/*********************************************************************************************************
  P1_4�����Ź�������
*********************************************************************************************************/
#define PIO1_4_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_4_AD5          0x01                                        /*  ADת��ͨ��5                 */
#define PIO1_4_CT32B1_MAT3  0x02                                        /*  32λ��ʱ��1ƥ�������ͨ��3  */

#define FUNC_PIO1_4         PIO1_4_GPIO

#define PIO1_4_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_4_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO1_4_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO1_4_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO1_4_MODE         PIO1_4_PULLUP

#define PIO1_4_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO1_4_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO1_4_HYS          PIO1_4_HYSDISABLE

#define PIO1_4_ANALOG       0                                           /*  ģ������ģʽ                */
#define PIO1_4_DIGITAL      1                                           /*  ����ģʽ                    */

#define PIO1_4_ADMODE       PIO1_4_DIGITAL

#define PIO1_4_CON         (FUNC_PIO1_4 | (PIO1_4_MODE << 3) | \
                           (PIO1_4_HYS << 5) | (PIO1_4_ADMODE << 7))
/*********************************************************************************************************
  P1_5�����Ź�������
*********************************************************************************************************/
#define PIO1_5_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_5_RTS          0x01                                        /*  UART����������            */
#define PIO1_5_CT32B0_CAP0  0x02                                        /*  32λ��ʱ��0�������룬ͨ��0  */

#define FUNC_PIO1_5         PIO1_5_GPIO

#define PIO1_5_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_5_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO1_5_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO1_5_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO1_5_MODE         PIO1_5_PULLUP

#define PIO1_5_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO1_5_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO1_5_HYS          PIO1_5_HYSDISABLE

#define PIO1_5_CON         (FUNC_PIO1_5 | (PIO1_5_MODE << 3) | (PIO1_5_HYS << 5))
/*********************************************************************************************************
  P1_6�����Ź�������
*********************************************************************************************************/
#define PIO1_6_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_6_UART_RXD     0x01                                        /*  UART���ݽ�������            */
#define PIO1_6_CT32B0_MAT0  0x02                                        /*  32λ��ʱ��0ƥ�������ͨ��0  */

#define FUNC_PIO1_6         PIO1_6_GPIO

#define PIO1_6_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_6_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO1_6_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO1_6_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO1_6_MODE         PIO1_6_PULLUP

#define PIO1_6_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO1_6_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO1_6_HYS          PIO1_6_HYSDISABLE

#define PIO1_6_CON         (FUNC_PIO1_6 | (PIO1_6_MODE << 3) | (PIO1_6_HYS << 5))
/*********************************************************************************************************
  P1_7�����Ź�������
*********************************************************************************************************/
#define PIO1_7_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_7_UART_TXD     0x01                                        /*  UART���ݷ�������            */
#define PIO1_7_CT32B0_MAT1  0x02                                        /*  32λ��ʱ��0ƥ�������ͨ��1  */

#define FUNC_PIO1_7         PIO1_7_GPIO
 
#define PIO1_7_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_7_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO1_7_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO1_7_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO1_7_MODE         PIO1_7_PULLUP

#define PIO1_7_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO1_7_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO1_7_HYS          PIO1_7_HYSDISABLE

#define PIO1_7_CON         (FUNC_PIO1_7 | (PIO1_7_MODE << 3) | (PIO1_7_HYS << 5))
/*********************************************************************************************************
  P1_8�����Ź�������
*********************************************************************************************************/
#define PIO1_8_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_8_CT16B1_CAP0  0x01                                        /*  16λ��ʱ��1�������룬ͨ��0  */

#define FUNC_PIO1_8         PIO1_8_GPIO

#define PIO1_8_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_8_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO1_8_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO1_8_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO1_8_MODE         PIO1_8_PULLUP

#define PIO1_8_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO1_8_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO1_8_HYS          PIO1_8_HYSDISABLE

#define PIO1_8_CON         (FUNC_PIO1_8 | (PIO1_8_MODE << 3) | (PIO1_8_HYS << 5))
/*********************************************************************************************************
  P1_9�����Ź�������
*********************************************************************************************************/
#define PIO1_9_GPIO         0x00                                        /*  GPIO                        */
#define PIO1_9_CT16B1_MAT0  0x01                                        /*  16λ��ʱ��1ƥ�������ͨ��0  */

#define FUNC_PIO1_9         PIO1_9_GPIO

#define PIO1_9_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_9_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO1_9_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO1_9_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO1_9_MODE         PIO1_9_PULLUP

#define PIO1_9_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO1_9_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO1_9_HYS          PIO1_9_HYSDISABLE

#define PIO1_9_CON          (FUNC_PIO1_9 | (PIO1_9_MODE << 3) | (PIO1_9_HYS << 5))
/*********************************************************************************************************
  P1_10�����Ź�������
*********************************************************************************************************/
#define PIO1_10_GPIO        0x00                                        /*  GPIO                        */
#define PIO1_10_AD6         0x01                                        /*  ADת��ͨ��6                 */
#define PIO1_10_CT16B1_MAT1 0x02                                        /*  16λ��ʱ��1ƥ�������ͨ��1  */

#define FUNC_PIO1_10        PIO1_10_GPIO

#define PIO1_10_INACTIVE    0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_10_PULLDOWN    0x01                                        /*  �ڲ�����                    */
#define PIO1_10_PULLUP      0x02                                        /*  �ڲ�����                    */
#define PIO1_10_REPEATER    0x03                                        /*  �м�ģʽ                    */

#define PIO1_10_MODE        PIO1_10_PULLUP

#define PIO1_10_HYSDISABLE  0                                           /*  �ͺ����                    */
#define PIO1_10_HYSENABLE   1                                           /*  �ͺ�ʹ��                    */

#define PIO1_10_HYS         PIO1_10_HYSDISABLE

#define PIO1_10_ANALOG      0                                           /*  ģ������ģʽ                */
#define PIO1_10_DIGITAL     1                                           /*  ����ģʽ                    */

#define PIO1_10_ADMODE      PIO1_10_DIGITAL

#define PIO1_10_CON         (FUNC_PIO1_10 | (PIO1_10_MODE << 3) | \
                            (PIO1_10_HYS << 5) | (PIO1_10_ADMODE << 7))
/*********************************************************************************************************
  P1_11�����Ź�������
*********************************************************************************************************/
#define PIO1_11_GPIO        0x00                                        /*  GPIO                        */
#define PIO1_11_AD7         0x01                                        /*  ADת��ͨ��7                 */

#define FUNC_PIO1_11        PIO1_11_GPIO

#define PIO1_11_INACTIVE    0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO1_11_PULLDOWN    0x01                                        /*  �ڲ�����                    */
#define PIO1_11_PULLUP      0x02                                        /*  �ڲ�����                    */
#define PIO1_11_REPEATER    0x03                                        /*  �м�ģʽ                    */

#define PIO1_11_MODE        PIO1_0_PULLUP

#define PIO1_11_HYSDISABLE  0                                           /*  �ͺ����                    */
#define PIO1_11_HYSENABLE   1                                           /*  �ͺ�ʹ��                    */

#define PIO1_11_HYS         PIO1_11_HYSDISABLE

#define PIO1_11_ANALOG      0                                           /*  ģ������ģʽ                */
#define PIO1_11_DIGITAL     1                                           /*  ����ģʽ                    */

#define PIO1_11_ADMODE      PIO1_11_DIGITAL

#define PIO1_11_CON        (FUNC_PIO1_11 | (PIO1_11_MODE << 3) | \
                           (PIO1_11_HYS << 5) | (PIO1_11_ADMODE << 7))
/*********************************************************************************************************
  P2_0�����Ź�������
*********************************************************************************************************/
#define PIO2_0_GPIO         0x00                                        /*  GPIO                        */
#define PIO2_0_DTR          0x01                                        /*  UART�ն˾���DTR             */
#define PIO2_0_SSEL1        0x02                                        /*  SSP���ߴӻ�ѡ��             */

#define FUNC_PIO2_0         PIO2_0_GPIO

#define PIO2_0_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO2_0_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO2_0_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO2_0_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO2_0_MODE         PIO2_0_PULLUP

#define PIO2_0_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO2_0_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */
 
#define PIO2_0_HYS          PIO2_0_HYSDISABLE

#define PIO2_0_CON         (FUNC_PIO2_0 | (PIO2_0_MODE << 3) | (PIO2_0_HYS << 5))
/*********************************************************************************************************
  P2_1�����Ź�������
  ע�⣺ ��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ��PLCC44��װ���д�����
*********************************************************************************************************/
#define PIO2_1_GPIO         0x00                                        /*  GPIO                        */
#define PIO2_1_DSR          0x01                                        /*  UART�������þ���DSR         */
#define PIO2_1_SCK1         0x02                                        /*  SSP����ʱ����               */

#define FUNC_PIO2_1         PIO2_1_GPIO

#define PIO2_1_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO2_1_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO2_1_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO2_1_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO2_1_MODE         PIO2_1_PULLUP

#define PIO2_1_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO2_1_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO2_1_HYS          PIO2_1_HYSDISABLE

#define PIO2_1_CON         (FUNC_PIO2_1 | (PIO2_1_MODE << 3) | (PIO2_1_HYS << 5))
/*********************************************************************************************************
  P2_2�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ��PLCC44��װ���д�����
*********************************************************************************************************/
#define PIO2_2_GPIO         0x00                                        /*  GPIO                        */
#define PIO2_2_DCD          0x01                                        /*  UART�����ز��������DCD     */
#define PIO2_2_MISO1        0x02                                        /*  SSP����������������       */

#define FUNC_PIO2_2         PIO2_2_GPIO

#define PIO2_2_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO2_2_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO2_2_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO2_2_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO2_2_MODE         PIO2_2_PULLUP

#define PIO2_2_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO2_2_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO2_2_HYS          PIO2_2_HYSDISABLE

#define PIO2_2_CON         (FUNC_PIO2_2 | (PIO2_2_MODE << 3) | (PIO2_2_HYS << 5))
/*********************************************************************************************************
  P2_3�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ��PLCC44��װ���д�����
*********************************************************************************************************/
#define PIO2_3_GPIO         0x00                                        /*  GPIO                        */
#define PIO2_3_RI           0x01                                        /*  UART�����ź�RI              */
#define PIO2_3_MOSI1        0x02                                        /*  SSP�����������������       */

#define FUNC_PIO2_3         PIO2_3_GPIO

#define PIO2_3_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO2_3_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO2_3_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO2_3_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO2_3_MODE         PIO2_3_PULLUP

#define PIO2_3_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO2_3_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO2_3_HYS          PIO2_3_HYSDISABLE

#define PIO2_3_CON         (FUNC_PIO2_3 | (PIO2_3_MODE << 3) | (PIO2_3_HYS << 5))
/*********************************************************************************************************
  P2_4�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ��PLCC44��װ���д�����
*********************************************************************************************************/
#define PIO2_4_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_4         PIO2_4_GPIO

#define PIO2_4_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO2_4_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO2_4_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO2_4_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO2_4_MODE         PIO2_4_PULLUP

#define PIO2_4_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO2_4_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO2_4_HYS          PIO2_4_HYSDISABLE

#define PIO2_4_CON         (FUNC_PIO2_4 | (PIO2_4_MODE << 3) | (PIO2_4_HYS << 5))
/*********************************************************************************************************
  P2_5�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ��PLCC44��װ���д�����
*********************************************************************************************************/
#define PIO2_5_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_5         PIO2_5_GPIO

#define PIO2_5_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO2_5_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO2_5_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO2_5_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO2_5_MODE         PIO2_5_PULLUP

#define PIO2_5_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO2_5_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO2_5_HYS          PIO2_5_HYSDISABLE

#define PIO2_5_CON         (FUNC_PIO2_5 | (PIO2_5_MODE << 3) | (PIO2_5_HYS << 5))
/*********************************************************************************************************
  P2_6�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ��PLCC44��װ���д�����
*********************************************************************************************************/
#define PIO2_6_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_6         PIO2_6_GPIO

#define PIO2_6_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO2_6_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO2_6_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO2_6_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO2_6_MODE         PIO2_6_PULLUP

#define PIO2_6_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO2_6_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO2_6_HYS          PIO2_6_HYSDISABLE

#define PIO2_6_CON         (FUNC_PIO2_6 | (PIO2_6_MODE << 3) | (PIO2_6_HYS << 5))
/*********************************************************************************************************
  P2_7�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ��PLCC44��װ���д�����
*********************************************************************************************************/
#define PIO2_7_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_7         PIO2_7_GPIO

#define PIO2_7_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO2_7_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO2_7_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO2_7_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO2_7_MODE         PIO2_7_PULLUP

#define PIO2_7_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO2_7_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO2_7_HYS          PIO2_7_HYSDISABLE

#define PIO2_7_CON         (FUNC_PIO2_7 | (PIO2_7_MODE << 3) | (PIO2_7_HYS << 5))
/*********************************************************************************************************
  P2_8�����Ź�������
  ��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ��PLCC44��װ���д�����
*********************************************************************************************************/
#define PIO2_8_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_8         PIO2_8_GPIO

#define PIO2_8_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO2_8_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO2_8_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO2_8_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO2_8_MODE         PIO2_8_PULLUP

#define PIO2_8_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO2_8_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO2_8_HYS          PIO2_8_HYSDISABLE

#define PIO2_8_CON         (FUNC_PIO2_8 | (PIO2_8_MODE << 3) | (PIO2_8_HYS << 5))
/*********************************************************************************************************
  P2_9�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ��PLCC44��װ���д�����
*********************************************************************************************************/
#define PIO2_9_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO2_9         PIO2_9_GPIO

#define PIO2_9_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO2_9_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO2_9_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO2_9_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO2_9_MODE         PIO2_9_PULLUP

#define PIO2_9_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO2_9_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO2_9_HYS          PIO2_9_HYSDISABLE

#define PIO2_9_CON         (FUNC_PIO2_9 | (PIO2_9_MODE << 3) | (PIO2_9_HYS << 5))
/*********************************************************************************************************
  P2_10�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ��PLCC44��װ���д�����
*********************************************************************************************************/
#define PIO2_10_GPIO        0x00                                        /*  GPIO                        */

#define FUNC_PIO2_10        PIO2_10_GPIO

#define PIO2_10_INACTIVE    0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO2_10_PULLDOWN    0x01                                        /*  �ڲ�����                    */
#define PIO2_10_PULLUP      0x02                                        /*  �ڲ�����                    */
#define PIO2_10_REPEATER    0x03                                        /*  �м�ģʽ                    */

#define PIO2_10_MODE        PIO2_10_PULLUP

#define PIO2_10_HYSDISABLE  0                                           /*  �ͺ����                    */
#define PIO2_10_HYSENABLE   1                                           /*  �ͺ�ʹ��                    */

#define PIO2_10_HYS         PIO2_10_HYSDISABLE

#define PIO2_10_CON        (FUNC_PIO2_10 | (PIO2_10_MODE << 3) | (PIO2_10_HYS << 5))
/*********************************************************************************************************
  P2_11�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ��PLCC44��װ���д�����
*********************************************************************************************************/
#define PIO2_11_GPIO        0x00                                        /*  GPIO                       */
#define PIO2_11_SCK0        0x01                                        /*  SSP��SCK����               */

#define FUNC_PIO2_11        PIO2_11_GPIO

#define PIO2_11_INACTIVE    0x00                                        /*  ���裨�Ȳ�����Ҳ��������   */
#define PIO2_11_PULLDOWN    0x01                                        /*  �ڲ�����                   */
#define PIO2_11_PULLUP      0x02                                        /*  �ڲ�����                   */
#define PIO2_11_REPEATER    0x03                                        /*  �м�ģʽ                   */

#define PIO2_11_MODE        PIO2_11_PULLUP

#define PIO2_11_HYSDISABLE  0                                           /*  �ͺ����                   */
#define PIO2_11_HYSENABLE   1                                           /*  �ͺ�ʹ��                   */

#define PIO2_11_HYS         PIO2_11_HYSDISABLE

#define PIO2_11_CON        (FUNC_PIO2_11 | (PIO2_11_MODE << 3) | (PIO2_11_HYS << 5))
/*********************************************************************************************************
  P3_0�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ���д�����
*********************************************************************************************************/
#define PIO3_0_GPIO         0x00                                        /*  GPIO                        */
#define PIO3_0_DTR          0x01                                        /*  UART�ն˾���DTR             */

#define FUNC_PIO3_0         PIO3_0_GPIO

#define PIO3_0_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO3_0_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO3_0_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO3_0_REPEATER     0x03                                        /*  �м�ģʽ                    */
 
#define PIO3_0_MODE         PIO3_0_PULLUP

#define PIO3_0_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO3_0_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO3_0_HYS          PIO3_0_HYSDISABLE

#define PIO3_0_CON         (FUNC_PIO3_0 | (PIO3_0_MODE << 3) | (PIO3_0_HYS << 5))
/*********************************************************************************************************
  P3_1�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ���д�����
*********************************************************************************************************/
#define PIO3_1_GPIO         0x00                                        /*  GPIO                        */
#define PIO3_1_DSR          0x01                                        /*  UART�������þ���DSR         */

#define FUNC_PIO3_1         PIO3_1_GPIO

#define PIO3_1_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO3_1_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO3_1_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO3_1_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO3_1_MODE         PIO3_1_PULLUP

#define PIO3_1_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO3_1_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO3_1_HYS          PIO3_1_HYSDISABLE

#define PIO3_1_CON         (FUNC_PIO3_1 | (PIO3_1_MODE << 3) | (PIO3_1_HYS << 5))
/*********************************************************************************************************
  P3_2�����Ź�������
  ע�⣺��LPC1111;LPC1112;LPC1113;LPC1114��HVQFN33��װ��LQFP48��װ���д�����
*********************************************************************************************************/
#define PIO3_2_GPIO         0x00                                        /*  GPIO                        */
#define PIO3_2_DCD          0x01                                        /*  UART�����ز��������DCD     */

#define FUNC_PIO3_2         PIO3_2_GPIO

#define PIO3_2_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO3_2_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO3_2_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO3_2_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO3_2_MODE         PIO3_2_PULLUP

#define PIO3_2_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO3_2_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO3_2_HYS          PIO3_2_HYSDISABLE

#define PIO3_2_CON         (FUNC_PIO3_2 | (PIO3_2_MODE << 3) | (PIO3_2_HYS << 5))
/*********************************************************************************************************
  P3_3�����Ź�������
  ע�⣺��LPC1113��LQFP48��װ;LPC1114��LQFP48��װ���д�����
*********************************************************************************************************/
#define PIO3_3_GPIO         0x00                                        /*  GPIO                        */
#define PIO3_3_RI           0x01                                        /*  UART�����ź�RI              */

#define FUNC_PIO3_3         PIO3_3_GPIO

#define PIO3_3_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO3_3_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO3_3_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO3_3_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO3_3_MODE         PIO3_3_PULLUP

#define PIO3_3_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO3_3_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO3_3_HYS          PIO3_3_HYSDISABLE

#define PIO3_3_CON         (FUNC_PIO3_3 | (PIO3_3_MODE << 3) | (PIO3_3_HYS << 5))
/*********************************************************************************************************
  P3_4�����Ź�������
*********************************************************************************************************/
#define PIO3_4_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO3_4         PIO3_4_GPIO

#define PIO3_4_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO3_4_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO3_4_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO3_4_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO3_4_MODE         PIO3_4_PULLUP

#define PIO3_4_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO3_4_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO3_4_HYS          PIO3_4_HYSDISABLE

#define PIO3_4_CON         (FUNC_PIO3_4 | (PIO3_4_MODE << 3) | (PIO3_4_HYS << 5))
/*********************************************************************************************************
  P3_5�����Ź�������
*********************************************************************************************************/
#define PIO3_5_GPIO         0x00                                        /*  GPIO                        */

#define FUNC_PIO3_5         PIO3_5_GPIO

#define PIO3_5_INACTIVE     0x00                                        /*  ���裨�Ȳ�����Ҳ��������    */
#define PIO3_5_PULLDOWN     0x01                                        /*  �ڲ�����                    */
#define PIO3_5_PULLUP       0x02                                        /*  �ڲ�����                    */
#define PIO3_5_REPEATER     0x03                                        /*  �м�ģʽ                    */

#define PIO3_5_MODE         PIO3_5_PULLUP

#define PIO3_5_HYSDISABLE   0                                           /*  �ͺ����                    */
#define PIO3_5_HYSENABLE    1                                           /*  �ͺ�ʹ��                    */

#define PIO3_5_HYS          PIO3_5_HYSDISABLE

#define PIO3_5_CON         (FUNC_PIO3_5 | (PIO3_5_MODE << 3) | (PIO3_5_HYS << 5))

/*********************************************************************************************************
** Function name:       pinInit
** Descriptions:        ��ʼ�����е��������ã�����������ӡ���������������
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
extern void pinInit(void);

#endif                                                                  /*  __LPC1300PINCFG_H           */
/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
