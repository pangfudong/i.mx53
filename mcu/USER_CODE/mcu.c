#include "mcu.h"
/*********************************************************************************************************
** Function name:       myDelay
** Descriptions:        软件延时
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void myDelay (INT32U ulTime)
{
   INT32U i;

   i = 0;
   while (ulTime--) {
       for (i = 0; i < 5000; i++);
   }
}

/*********************************************************************************************************
** Function name:       myDelay
** Descriptions:        LED闪烁
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void onLight500(void)
{
    GPIO2DIR |= (1ul << 1); 
    while(1)
    {
        GPIO2DATA &=  ~(1ul << 1);
        myDelay(500);
        GPIO2DATA |=  (1ul << 1);
        myDelay(500);
    }
}

/*********************************************************************************************************
** Function name:       ADCInit
** Descriptions:        ADC初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void ADCInit( void )
{
    PDRUNCFG      &= ~(0x01 << 4);                                      /*  ADC模块上电                  */
    SYSAHBCLKCTRL |=  (0x01 << 13);                                     /*  使能ADC模块时钟              */

    IOCON_PIO0_11 &= ~0xBF;                                             /*  配置PIO0_11为模拟输入模式    */
    IOCON_PIO0_11 |=  0x02;                                             /*  PIO0_11模拟输入通道0         */

    AD0CR = ( 0x01 << 0 ) |                                             /*  SEL=1,选择ADC0               */
            ((FAHBCLK / 1000000 - 1) << 8 ) |                           /*  转换时钟1MHz                 */
            ( 0 << 16 ) |                                               /*  软件控制转换操作             */
            ( 0 << 17 ) |                                               /*  使用11 clocks转换            */
            ( 0 << 24 ) |                                               /*  ADC转换停止                  */
            ( 0 << 27 );                                                /*  直接启动ADC转换，此位无效    */
}

/*********************************************************************************************************
** Function name:       uartInit
** Descriptions:        串口初始化，设置为8位数据位，1位起始位，1位停止位，无奇偶校验，波特率为19200
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartInit (void)
{
    INT16U usFdiv;

    IOCON_PIO1_6  &= ~0x07;
    IOCON_PIO1_6  |= (1 << 0);                                          /*  配置P1.6为RXD               */
    IOCON_PIO1_7  &= ~0x07;
    IOCON_PIO1_7  |= (1 << 0);                                          /*  配置P1.7为TXD               */
    SYSAHBCLKCTRL |= (1 << 12);                                         /*  打开UART功能部件时钟        */
    UARTCLKDIV     = 0x01;                                              /*  UART时钟分频                */
    
    U0LCR  = 0x83;                                                      /*  允许设置波特率              */
    usFdiv = (FAHBCLK / UARTCLKDIV / 16) / 19200;                       /*  设置波特率                  */
    U0DLM  = usFdiv / 256;
    U0DLL  = usFdiv % 256; 
    U0LCR  = 0x03;                                                      /*  锁定波特率                  */
    U0FCR  = 0x07;
}

/*********************************************************************************************************
** Function name:       uartRecvByte
** Descriptions:        从串口接受子节数据，并等待数据接受完成，使用查询方式
** input parameters:    ucDat:   要接收的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartRecvByte (INT8U *ucDat)
{
    *ucDat = U0RBR;                                                      /*  读出数据                    */
    while ((U0LSR & 0x40) == 0){                                        /*  等待数据接收完毕            */
	}
}

/*********************************************************************************************************
** Function name:       uartRecvStr
** Descriptions:        向串口接受字符串
** input parameters:    puiStr:   要接受的字符串指针
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartRecvStr (INT8U *pucStr)
{
    while (1){
        if (*pucStr == '\0') {
		    break;                                                      /*  遇到结束符，退出            */
		}
        uartRecvByte (pucStr++);
    }
}

/*********************************************************************************************************
** Function name:       sleep
** Descriptions:        让触摸屏sleep，输出高电平
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void sleep (void)
{
    SET_GPIO_syb_n_bit_1(DIR,0,0)                     /*  输出数据                    */
    SET_GPIO_syb_n_bit_1(DATA,0,0)                    /*  输出高电平                    */
}

/*********************************************************************************************************
** Function name:       active
** Descriptions:        让触摸屏active，输出低电平
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void active (void)
{
    SET_GPIO_syb_n_bit_1(DIR,0,1)                     /*  输出数据                    */
    SET_GPIO_syb_n_bit_0(DATA,0,1)                    /*  输出低电平                    */
}

/*********************************************************************************************************
** Function name:       reset
** Descriptions:        reset触摸屏，需要在上电之后，先输出低电平100ms，再输出高电平
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void reset (void)
{
    SET_GPIO_syb_n_bit_1(DIR,2,6)                     /*  输出数据                    */
    SET_GPIO_syb_n_bit_0(DATA,2,6)                    /*  输出低电平                    */
    myDelay(100);                                     /*  保持100ms                    */
    SET_GPIO_syb_n_bit_1(DATA,2,6)                    /*  输出高电平                    */
}

/*********************************************************************************************************
** Function name:       walTopInit
** Descriptions:        walTopInit触摸屏启动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void walTopInit (void)
{
    active();
    reset();
}
