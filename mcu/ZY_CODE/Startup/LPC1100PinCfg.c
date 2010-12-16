/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:            LPC1100PinCfg.c
** Last modified Date:   2009-05-12
** Last Version:         V1.00
** Descriptions:         ��������
**
**--------------------------------------------------------------------------------------------------------
** Created by:           lanwuqiang
** Created date:         2009-10-22
** Version:              V1.00
** Descriptions:         
**--------------------------------------------------------------------------------------------------------
** Modified by:          
** Modified date:        
** Version:              
** Descriptions:         
*********************************************************************************************************/
#include "..\..\config.h"

/*********************************************************************************************************
** Function name:      pinInit
** Descriptions:       ���ų�ʼ������
**                     ����ʼ�����ж��û����������ŵ�����,�����������,��������������
** input parameters:   none
** output parameters:  none
** Returned value:     none
** Created by:         Lanwuqiang
** Created Date:       2010-02-23
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
*********************************************************************************************************/
void pinInit (void)
{
    IOCON_PIO0_0  = PIO0_0_CON;                                         /*  PIO0_0 ��������             */
    IOCON_PIO0_1  = PIO0_1_CON;                                         /*  PIO0_1 ��������             */
    IOCON_PIO0_2  = PIO0_2_CON;                                         /*  PIO0_2 ��������             */
    IOCON_PIO0_3  = PIO0_3_CON;                                         /*  PIO0_3 ��������             */
    IOCON_PIO0_4  = PIO0_4_CON;                                         /*  PIO0_4 ��������             */
    IOCON_PIO0_5  = PIO0_5_CON;                                         /*  PIO0_5 ��������             */
    IOCON_PIO0_6  = PIO0_6_CON;                                         /*  PIO0_6 ��������             */
    IOCON_PIO0_7  = PIO0_7_CON;                                         /*  PIO0_7 ��������             */
    IOCON_PIO0_8  = PIO0_8_CON;                                         /*  PIO0_8 ��������             */
    IOCON_PIO0_9  = PIO0_9_CON;                                         /*  PIO0_9 ��������             */
    IOCON_PIO0_10 = PIO0_10_CON;                                        /*  PIO0_10��������             */
    IOCON_PIO0_11 = PIO0_11_CON;                                        /*  PIO0_11��������             */

    IOCON_PIO1_0  = PIO1_0_CON;                                         /*  PIO1_0 ��������             */
    IOCON_PIO1_1  = PIO1_1_CON;                                         /*  PIO1_1 ��������             */
    IOCON_PIO1_2  = PIO1_2_CON;                                         /*  PIO1_2 ��������             */
    IOCON_PIO1_3  = PIO1_3_CON;                                         /*  PIO1_3 ��������             */
    IOCON_PIO1_4  = PIO1_4_CON;                                         /*  PIO1_4 ��������             */
    IOCON_PIO1_5  = PIO1_5_CON;                                         /*  PIO1_5 ��������             */
    IOCON_PIO1_6  = PIO1_6_CON;                                         /*  PIO1_6 ��������             */
    IOCON_PIO1_7  = PIO1_7_CON;                                         /*  PIO1_7 ��������             */
    IOCON_PIO1_8  = PIO1_8_CON;                                         /*  PIO1_8 ��������             */
    IOCON_PIO1_9  = PIO1_9_CON;                                         /*  PIO1_9 ��������             */
    IOCON_PIO1_10 = PIO1_10_CON;                                        /*  PIO1_10��������             */
    IOCON_PIO1_11 = PIO1_11_CON;                                        /*  PIO1_11��������             */

    IOCON_PIO2_0  = PIO2_0_CON;                                         /*  PIO2_0 ��������             */
    IOCON_PIO2_1  = PIO2_1_CON;                                         /*  PIO2_1 ��������             */
    IOCON_PIO2_2  = PIO2_2_CON;                                         /*  PIO2_2 ��������             */
    IOCON_PIO2_3  = PIO2_3_CON;                                         /*  PIO2_3 ��������             */
    IOCON_PIO2_4  = PIO2_4_CON;                                         /*  PIO2_4 ��������             */
    IOCON_PIO2_5  = PIO2_5_CON;                                         /*  PIO2_5 ��������             */
    IOCON_PIO2_6  = PIO2_6_CON;                                         /*  PIO2_6 ��������             */
    IOCON_PIO2_7  = PIO2_7_CON;                                         /*  PIO2_7 ��������             */
    IOCON_PIO2_8  = PIO2_8_CON;                                         /*  PIO2_8 ��������             */
    IOCON_PIO2_9  = PIO2_9_CON;                                         /*  PIO2_9 ��������             */
    IOCON_PIO2_10 = PIO2_10_CON;                                        /*  PIO2_10��������             */
    IOCON_PIO2_11 = PIO2_11_CON;                                        /*  PIO2_11��������             */

    IOCON_PIO3_0  = PIO3_0_CON;                                         /*  PIO3_0 ��������             */
    IOCON_PIO3_1  = PIO3_1_CON;                                         /*  PIO3_1 ��������             */
    IOCON_PIO3_2  = PIO3_2_CON;                                         /*  PIO3_2 ��������             */
    IOCON_PIO3_3  = PIO3_3_CON;                                         /*  PIO3_3 ��������             */
    IOCON_PIO3_4  = PIO3_4_CON;                                         /*  PIO3_4 ��������             */
    IOCON_PIO3_5  = PIO3_5_CON;                                         /*  PIO3_5 ��������             */
}

/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
