#ifndef __MCU_H 
#define __MCU_H

#include "..\config.h"

/*********************************************************************************************************
** Function name:       myDelay
** Descriptions:        软件延时
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
extern void myDelay (INT32U ulTime);

/*********************************************************************************************************
** Function name:       myDelay
** Descriptions:        LED闪烁
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
extern void onLight500 (void);

/*********************************************************************************************************
** Function name:       ADCInit
** Descriptions:        ADC初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
extern void ADCInit (void);

/*********************************************************************************************************
** Function name:       uartInit
** Descriptions:        串口初始化，设置为8位数据位，1位起始位，1位停止位，无奇偶校验，波特率为19200
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
extern void uartInit (void);

/*********************************************************************************************************
** Function name:       uartRecvByte
** Descriptions:        从串口接受子节数据，并等待数据接受完成，使用查询方式
** input parameters:    ucDat:   要接收的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
extern void uartRecvByte (INT8U *ucDat);

/*********************************************************************************************************
** Function name:       uartRecvStr
** Descriptions:        向串口接受字符串
** input parameters:    puiStr:   要接受的字符串指针
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
extern void uartRecvStr (INT8U *pucStr);

/*********************************************************************************************************
** Function name:       sleep
** Descriptions:        让触摸屏sleep，输出高电平
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
extern void sleep (void);

/*********************************************************************************************************
** Function name:       active
** Descriptions:        让触摸屏active，输出低电平
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
extern void active (void);

/*********************************************************************************************************
** Function name:       reset
** Descriptions:        reset触摸屏，需要在上电之后，先输出低电平100ms，再输出高电平
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
extern void reset (void);

/*********************************************************************************************************
** Function name:       walTopInit
** Descriptions:        walTopInit触摸屏启动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
extern void walTopInit (void);
#endif                                                                  /*  __WALTOP_H                  */
