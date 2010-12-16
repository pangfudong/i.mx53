/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           main.c
** Last modified Date:  2010-02-04
** Last Version:        V1.0
** Descriptions:        The main() function example template
**
**--------------------------------------------------------------------------------------------------------
** Created by:          Lanwuqiang
** Created date:        2010-02-05
** Version:             V1.0
** Descriptions:        添加用户应用程序
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         ZhangNingbo
** Modified date:       2010-02-25
** Version:             V1.0
** Descriptions:        GPIO中断示例程序
**
**--------------------------------------------------------------------------------------------------------
** Modified by:        
** Modified date:      
** Version:            
** Descriptions:       
**
** Rechecked by:
*********************************************************************************************************/
#include "..\config.h"
#include "stdio.h"
/*********************************************************************************************************
  宏定义
*********************************************************************************************************/
#define BEEP        (1ul << 7)
#define BEEP_INIT()  GPIO2DIR  |=  BEEP                                 /*  蜂鸣器初始化                */
#define BEEPOFF()    GPIO2DATA |=  BEEP                                 /*  蜂鸣器关                    */
#define BEEPON()     GPIO2DATA &= ~BEEP                                 /*  蜂鸣器开                    */

#define LED         (1ul << 1)                                          /*  LED定义PIO2_1              */

#define LED_DIR(x,y)    GPIO##x##DIR  |= (1<<y);
#define LED_ON(x,y)     GPIO##x##DATA &= ~(1<<y); 
#define LED_OFF(x,y)    GPIO##x##DATA |= (1<<y); 

#define SET_GPIO_n_syb_bit_0(n,syb,bit)     GPIO##n##syb &= ~(1<<bit);
#define SET_GPIO_n_syb_bit_1(n,syb,bit)     GPIO##n##syb |=  (1<<bit);
#define SET_GPIO_n_syb_value(n,syb,value)   GPIO##n##syb =   (value);

//#define  UART_BPS    115200                                             /*  串口通信波特率              */
char     GcRcvBuf[20] = {0};                                                  /*  AD采集到的数据              */
char     keystate = 0; // 0--up 1--down
char     keyinit = 1; // 0--false 1--true
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
** Descriptions:        串口初始化，设置为8位数据位，1位停止位，无奇偶校验，波特率为115200
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
    usFdiv = (FAHBCLK / UARTCLKDIV / 16) / UART_BPS;                    /*  设置波特率                  */
    U0DLM  = usFdiv / 256;
    U0DLL  = usFdiv % 256; 
    U0LCR  = 0x03;                                                      /*  锁定波特率                  */
    U0FCR  = 0x07;
}

/*********************************************************************************************************
** Function name:       uartSendByte
** Descriptions:        向串口发送子节数据，并等待数据发送完成，使用查询方式
** input parameters:    ucDat:   要发送的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartSendByte (INT8U ucDat)
{
    U0THR = ucDat;                                                      /*  写入数据                    */
    while ((U0LSR & 0x40) == 0){                                        /*  等待数据发送完毕            */
	}
}

/*********************************************************************************************************
** Function name:       uartSendStr
** Descriptions:        向串口发送字符串
** input parameters:    puiStr:   要发送的字符串指针
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartSendStr (INT8U  const *pucStr)
{
    while (1){
        if (*pucStr == '\0') {
		    break;                                                      /*  遇到结束符，退出            */
		}
        uartSendByte (*pucStr++);
    }
}

/*********************************************************************************************************
** Function name:       pcDispChar
** Descriptions:        向PC机发送显示字符
** input parameters:    x:        显示字符的横坐标
**                      y:        显示字符的纵坐标
**                      ucChr:    显示的字符，不能为ff
**                      ucColor:  显示的状态，包括前景色、背景色、闪烁位。
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void pcDispChar (INT8U x, INT8U y, INT8U ucChr, INT8U ucColor)
{
    uartSendByte(0xff);                                                 /*  起始字符                    */
    uartSendByte(x);
    uartSendByte(y);
    uartSendByte(ucChr);
    //uartSendByte(ucColor);//由于pc端没能支持 暂时关闭颜色设置
}

/*********************************************************************************************************
** Function name:       iSendStr
** Descriptions:        向上位机发送字符串
** input parameters:    x:        显示字符的横坐标
**                      y:        显示字符的纵坐标
**                      ucColor:  显示的状态，包括前景色、背景色、闪烁位。
**                                与DOS字符显示一样：0～3,前景色，4～6，背景色，7，闪烁位
**                      pcStr:   要发送的字符串，以'\0'结束
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void iSendStr (INT8U x, INT8U y, INT8U ucColor, char *pcStr)
{
    while (1) {
        if (*pcStr == '\0') {                                           /*  结束字符                    */
            break;
        }
        pcDispChar(x++, y, *pcStr++, ucColor);
        if (x >= 80) {
            x = 0;
            y++;
        }
    }
}

/*********************************************************************************************************
** Function name:       GPIOIsr
** Descriptions:        GPIO中断服务函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/

void onLight500()
{
    GPIO2DIR |= LED; 
    while(1)
    {
        GPIO2DATA &=  ~LED;
        myDelay(500);
        GPIO2DATA |=  LED;
        myDelay(500);
    }
}

void onLight2000()
{
    GPIO2DIR |= LED; 
    while(1)
    {
        GPIO2DATA &=  ~LED;//~(1<<1);
        myDelay(2000);
        GPIO2DATA |= LED;//(1<<1);
        myDelay(2000);
    }
}

// void GPIOIsr (void)
// {
   // GPIO1IC |= KEY; 
    // GPIO2IC |= KEY;
    // onLight500(); 
// }
    
    


void onKeyDown (void)
{
     iSendStr(0, 0, 0x30, "onKeyDown !! \r\n");                                 /*  将数据发送到串口显示        */ 
}

void onKeyUp (void)
{
     iSendStr(0, 0, 0x30, "onKeyUp !! \r\n");                                 /*  将数据发送到串口显示        */ 
}

void intrufunc (void)
{
    GPIO0IC |= (1<<2);
    
    //SET_GPIO_n_syb_bit_1(0,IBE,2);
    //if (GPIO0IEV & (1<<2) == 1)
    
    if (keyinit == 1) 
    {
        if (keystate == 0)
            keystate = 1; 
        else 
            keystate = 0;
        
        keyinit = 0;
    }

    if (keystate == 0) // keyup
    //if ( (GPIO0IEV & (1<<2)) == (1<<2)) will be work, need to test, then delete keystate and keyinit
    {
        onKeyUp();
             // 设置P0.2 下边沿中断 keydown
     SET_GPIO_n_syb_bit_0(0,DIR,2);
     SET_GPIO_n_syb_value(0,IS,0x00);
     SET_GPIO_n_syb_bit_0(0,IEV,2);
     SET_GPIO_n_syb_bit_1(0,IE,2);
keystate = 1;
        
    }
    else
    {
        onKeyDown();
             // 设置P0.2 上边沿中断 keyup
     SET_GPIO_n_syb_bit_0(0,DIR,2);
     SET_GPIO_n_syb_value(0,IS,0x00);
     SET_GPIO_n_syb_bit_1(0,IEV,2);
     SET_GPIO_n_syb_bit_1(0,IE,2);
keystate = 0;
        
    }GPIO0IC |= (1<<2);
    
         // sprintf(GcRcvBuf, "IEV= %s\r\n", GPIO0IEV);
     // iSendStr(0, 0, 0x30, GcRcvBuf);
    
}
/*********************************************************************************************************
** Function name:       main
** Descriptions:        当硬件基于EasyCortexM3-1300开发板时，需要将P2.7与BEEP短接，P3.0与KEY1短接，每按下
**                      一次按键蜂鸣器响一声。
**                      用户也可以根据要求自行设计底板连接。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
int main (void)
{
    targetInit();                                                       /*  初始化目标板，切勿删除      */
    pinInit();                                                          /*  引脚初始化                  */
    uartInit();
    SYSAHBCLKCTRL |= (1ul << 6);                                        /*  使能GPIO模块时钟            */

    iSendStr(0, 0, 0x30, "go !! \r\n");
    // BEEP_INIT();
    // GPIO3DIR &= ~KEY;                                                   /*  设置P3.0为输入              */
    // GPIO3IS   = 0x00;                                                   /*  P3.0为边沿中断              */
    // GPIO3IEV &= ~KEY;                                                    /*  下升沿中断                  */
    // GPIO3IE  |= KEY;                                                    /*  P3.0中断不屏蔽              */
    // zyIsrSet(NVIC_PIOINT3, (unsigned long)GPIOIsr, PRIO_ONE);
    
    
    
    // 先设置P0.2 下边沿中断 keydown
     SET_GPIO_n_syb_bit_0(0,DIR,2);
     SET_GPIO_n_syb_value(0,IS,0x00);
     SET_GPIO_n_syb_bit_0(0,IEV,2);
     SET_GPIO_n_syb_bit_1(0,IE,2);
        
    zyIsrSet(NVIC_PIOINT0, (unsigned long)intrufunc, PRIO_ONE);
    onLight2000();
   while (1) ;
}
// int main (void)
// {
    // INT32U  i;
    // INT32U  ulADCData; 
    // INT32U  ulADCBuf;

    // targetInit();                                                       /*  初始化目标板，切勿删除      */
    // pinInit();

    // uartInit();                                                         /*  串口初始化                  */
    //ADCInit();                                                          /*  ADC模块初始化               */


    //iSendStr(0, 0, 0x30, GcRcvBuf);                                 /*  将数据发送到串口显示        */
    // myDelay(2000);
    // }
// }
/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
