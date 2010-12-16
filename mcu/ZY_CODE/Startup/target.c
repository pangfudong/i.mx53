/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           target.c
** Last modified Date:  2009-07-22
** Last Version:        V1.00
** Descriptions:        the specific codes for LPC1700 target boards
**                      Every project should include a copy of this file, user may modify it as needed
**--------------------------------------------------------------------------------------------------------
** Created by:          chenmingji
** Created date:        2009-07-22
** Version:             V1.00
** Descriptions:        The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         Lanwuqiang
** Modified date:       2009-02-04
** Version:             V1.01
** Descriptions:        ��ֲ��LPC1100
**
*********************************************************************************************************/
#include  "..\..\config.h"

/*********************************************************************************************************
** Function name:           defaultVectorHandle
** Descriptions:            Ĭ���쳣�������
** input parameters:        none
** output parameters:       none
** Returned value:          none
*********************************************************************************************************/
void defaultVectorHandle (void)
{
#ifdef DEBUG 
    while (1);
#else
    zyReset(ZY_HARD_RESET);
#endif                                                                  /*  DEBUG                       */
}

/*********************************************************************************************************
** Function name:           targetInit
** Descriptions:            Initialize the target
** input parameters:        none
** output parameters:       none
** Returned value:          none
*********************************************************************************************************/
void targetInit (void)
{
    INT32U i, uiRegVal;
    extern void *__GpvVectorTable[];
    zyIrqDisable();
    
    SYSMEMREMAP = 0x02;                                                 /*  �쳣��������ӳ��            */
    VTOR        = (unsigned long)__GpvVectorTable;
    SYSOSCCTRL  = 0x00;                                                 /*  ����Ƶ�ʹ�����Χѡ��      */

    PDRUNCFG &= ~(0x1ul << 5);                                          /*  ϵͳ�����ϵ�              */
    for ( i = 0; i < 0x100; i++ ) {                                     /*  �ȴ������ȶ�              */
    }
    SYSPLLCLKSEL = MAIN_CLKSRCSEL_VALUE;                                /*  ѡ��ϵͳ����OSC           */
    SYSPLLCLKUEN = 0x00;                                                /*  �л�ʱ��Դ                  */
    SYSPLLCLKUEN = 0x01;                                                /*  ����ʱ��Դ                  */
    while (!(SYSPLLCLKUEN & 0x01)) {                                    /*  �ȴ��������                */	
    }
    uiRegVal   = SYSPLLCTRL;
    uiRegVal  &= ~0x1FF;
    SYSPLLCTRL = (uiRegVal | (PLL_PVALUE << 5) | PLL_MVALUE);           /*  Ԥ��Ƶ��M+1 �� 2*P          */
    PDRUNCFG  &= ~(0x01ul << 7);                                        /*  ϵͳMAIN PLL�ϵ�            */
    while (!(SYSPLLSTAT & 0x01)){                                       /*  �ȴ�����                    */
    }
    MAINCLKSEL = 0x03;                                                  /*  ѡ��PLL���                 */
    MAINCLKUEN = 0x01;                                                  /*  ����MCLKʱ��Դѡ��          */
    MAINCLKUEN = 0x00;                                                  /*  ��ת���¼Ĵ���              */
    MAINCLKUEN = 0x01;
    while (!(MAINCLKUEN & 0x01)) {                                      /*  �ȴ��������                */
    }
    SYSAHBCLKDIV = SYS_AHB_DIV_VALUE;                                   /*  SYStem AHBʱ�ӷ�Ƶ          */
    
    SYSAHBCLKCTRL |= (1ul << 16);                                       /*  ʹ��AHB����ʱ��             */
                                                                        /*  �����޷�����ICON            */
    zyIrqEnable();
    zyIfInit();
    return;
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
