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
** Descriptions:        移植于LPC1100
**
*********************************************************************************************************/
#include  "..\..\config.h"

/*********************************************************************************************************
** Function name:           defaultVectorHandle
** Descriptions:            默认异常处理程序
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
    
    SYSMEMREMAP = 0x02;                                                 /*  异常向量表重映射            */
    VTOR        = (unsigned long)__GpvVectorTable;
    SYSOSCCTRL  = 0x00;                                                 /*  振荡器频率工作范围选择      */

    PDRUNCFG &= ~(0x1ul << 5);                                          /*  系统振荡器上电              */
    for ( i = 0; i < 0x100; i++ ) {                                     /*  等待振荡器稳定              */
    }
    SYSPLLCLKSEL = MAIN_CLKSRCSEL_VALUE;                                /*  选择系统振荡器OSC           */
    SYSPLLCLKUEN = 0x00;                                                /*  切换时钟源                  */
    SYSPLLCLKUEN = 0x01;                                                /*  更新时钟源                  */
    while (!(SYSPLLCLKUEN & 0x01)) {                                    /*  等待更新完成                */	
    }
    uiRegVal   = SYSPLLCTRL;
    uiRegVal  &= ~0x1FF;
    SYSPLLCTRL = (uiRegVal | (PLL_PVALUE << 5) | PLL_MVALUE);           /*  预分频：M+1 与 2*P          */
    PDRUNCFG  &= ~(0x01ul << 7);                                        /*  系统MAIN PLL上电            */
    while (!(SYSPLLSTAT & 0x01)){                                       /*  等待锁定                    */
    }
    MAINCLKSEL = 0x03;                                                  /*  选择PLL输出                 */
    MAINCLKUEN = 0x01;                                                  /*  更新MCLK时钟源选择          */
    MAINCLKUEN = 0x00;                                                  /*  翻转更新寄存器              */
    MAINCLKUEN = 0x01;
    while (!(MAINCLKUEN & 0x01)) {                                      /*  等待更新完成                */
    }
    SYSAHBCLKDIV = SYS_AHB_DIV_VALUE;                                   /*  SYStem AHB时钟分频          */
    
    SYSAHBCLKCTRL |= (1ul << 16);                                       /*  使能AHB总线时钟             */
                                                                        /*  否则无法配置ICON            */
    zyIrqEnable();
    zyIfInit();
    return;
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
