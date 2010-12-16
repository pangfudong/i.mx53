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
** Descriptions:        ����û�Ӧ�ó���
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         ZhangNingbo
** Modified date:       2010-02-25
** Version:             V1.0
** Descriptions:        GPIO�ж�ʾ������
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
  �궨��
*********************************************************************************************************/
#define BEEP        (1ul << 7)
#define BEEP_INIT()  GPIO2DIR  |=  BEEP                                 /*  ��������ʼ��                */
#define BEEPOFF()    GPIO2DATA |=  BEEP                                 /*  ��������                    */
#define BEEPON()     GPIO2DATA &= ~BEEP                                 /*  ��������                    */

#define LED         (1ul << 1)                                          /*  LED����PIO2_1              */

#define LED_DIR(x,y)    GPIO##x##DIR  |= (1<<y);
#define LED_ON(x,y)     GPIO##x##DATA &= ~(1<<y); 
#define LED_OFF(x,y)    GPIO##x##DATA |= (1<<y); 

#define SET_GPIO_n_syb_bit_0(n,syb,bit)     GPIO##n##syb &= ~(1<<bit);
#define SET_GPIO_n_syb_bit_1(n,syb,bit)     GPIO##n##syb |=  (1<<bit);
#define SET_GPIO_n_syb_value(n,syb,value)   GPIO##n##syb =   (value);

//#define  UART_BPS    115200                                             /*  ����ͨ�Ų�����              */
char     GcRcvBuf[20] = {0};                                                  /*  AD�ɼ���������              */
char     keystate = 0; // 0--up 1--down
char     keyinit = 1; // 0--false 1--true
/*********************************************************************************************************
** Function name:       myDelay
** Descriptions:        �����ʱ
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        ADC��ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void ADCInit( void )
{
    PDRUNCFG      &= ~(0x01 << 4);                                      /*  ADCģ���ϵ�                  */
    SYSAHBCLKCTRL |=  (0x01 << 13);                                     /*  ʹ��ADCģ��ʱ��              */

    IOCON_PIO0_11 &= ~0xBF;                                             /*  ����PIO0_11Ϊģ������ģʽ    */
    IOCON_PIO0_11 |=  0x02;                                             /*  PIO0_11ģ������ͨ��0         */

    AD0CR = ( 0x01 << 0 ) |                                             /*  SEL=1,ѡ��ADC0               */
            ((FAHBCLK / 1000000 - 1) << 8 ) |                           /*  ת��ʱ��1MHz                 */
            ( 0 << 16 ) |                                               /*  �������ת������             */
            ( 0 << 17 ) |                                               /*  ʹ��11 clocksת��            */
            ( 0 << 24 ) |                                               /*  ADCת��ֹͣ                  */
            ( 0 << 27 );                                                /*  ֱ������ADCת������λ��Ч    */
}
/*********************************************************************************************************
** Function name:       uartInit
** Descriptions:        ���ڳ�ʼ��������Ϊ8λ����λ��1λֹͣλ������żУ�飬������Ϊ115200
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void uartInit (void)
{
    INT16U usFdiv;

    IOCON_PIO1_6  &= ~0x07;
    IOCON_PIO1_6  |= (1 << 0);                                          /*  ����P1.6ΪRXD               */
    IOCON_PIO1_7  &= ~0x07;
    IOCON_PIO1_7  |= (1 << 0);                                          /*  ����P1.7ΪTXD               */
    SYSAHBCLKCTRL |= (1 << 12);                                         /*  ��UART���ܲ���ʱ��        */
    UARTCLKDIV     = 0x01;                                              /*  UARTʱ�ӷ�Ƶ                */
    
    U0LCR  = 0x83;                                                      /*  �������ò�����              */
    usFdiv = (FAHBCLK / UARTCLKDIV / 16) / UART_BPS;                    /*  ���ò�����                  */
    U0DLM  = usFdiv / 256;
    U0DLL  = usFdiv % 256; 
    U0LCR  = 0x03;                                                      /*  ����������                  */
    U0FCR  = 0x07;
}

/*********************************************************************************************************
** Function name:       uartSendByte
** Descriptions:        �򴮿ڷ����ӽ����ݣ����ȴ����ݷ�����ɣ�ʹ�ò�ѯ��ʽ
** input parameters:    ucDat:   Ҫ���͵�����
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void uartSendByte (INT8U ucDat)
{
    U0THR = ucDat;                                                      /*  д������                    */
    while ((U0LSR & 0x40) == 0){                                        /*  �ȴ����ݷ������            */
	}
}

/*********************************************************************************************************
** Function name:       uartSendStr
** Descriptions:        �򴮿ڷ����ַ���
** input parameters:    puiStr:   Ҫ���͵��ַ���ָ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void uartSendStr (INT8U  const *pucStr)
{
    while (1){
        if (*pucStr == '\0') {
		    break;                                                      /*  �������������˳�            */
		}
        uartSendByte (*pucStr++);
    }
}

/*********************************************************************************************************
** Function name:       pcDispChar
** Descriptions:        ��PC��������ʾ�ַ�
** input parameters:    x:        ��ʾ�ַ��ĺ�����
**                      y:        ��ʾ�ַ���������
**                      ucChr:    ��ʾ���ַ�������Ϊff
**                      ucColor:  ��ʾ��״̬������ǰ��ɫ������ɫ����˸λ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void pcDispChar (INT8U x, INT8U y, INT8U ucChr, INT8U ucColor)
{
    uartSendByte(0xff);                                                 /*  ��ʼ�ַ�                    */
    uartSendByte(x);
    uartSendByte(y);
    uartSendByte(ucChr);
    //uartSendByte(ucColor);//����pc��û��֧�� ��ʱ�ر���ɫ����
}

/*********************************************************************************************************
** Function name:       iSendStr
** Descriptions:        ����λ�������ַ���
** input parameters:    x:        ��ʾ�ַ��ĺ�����
**                      y:        ��ʾ�ַ���������
**                      ucColor:  ��ʾ��״̬������ǰ��ɫ������ɫ����˸λ��
**                                ��DOS�ַ���ʾһ����0��3,ǰ��ɫ��4��6������ɫ��7����˸λ
**                      pcStr:   Ҫ���͵��ַ�������'\0'����
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void iSendStr (INT8U x, INT8U y, INT8U ucColor, char *pcStr)
{
    while (1) {
        if (*pcStr == '\0') {                                           /*  �����ַ�                    */
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
** Descriptions:        GPIO�жϷ�����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
     iSendStr(0, 0, 0x30, "onKeyDown !! \r\n");                                 /*  �����ݷ��͵�������ʾ        */ 
}

void onKeyUp (void)
{
     iSendStr(0, 0, 0x30, "onKeyUp !! \r\n");                                 /*  �����ݷ��͵�������ʾ        */ 
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
             // ����P0.2 �±����ж� keydown
     SET_GPIO_n_syb_bit_0(0,DIR,2);
     SET_GPIO_n_syb_value(0,IS,0x00);
     SET_GPIO_n_syb_bit_0(0,IEV,2);
     SET_GPIO_n_syb_bit_1(0,IE,2);
keystate = 1;
        
    }
    else
    {
        onKeyDown();
             // ����P0.2 �ϱ����ж� keyup
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
** Descriptions:        ��Ӳ������EasyCortexM3-1300������ʱ����Ҫ��P2.7��BEEP�̽ӣ�P3.0��KEY1�̽ӣ�ÿ����
**                      һ�ΰ�����������һ����
**                      �û�Ҳ���Ը���Ҫ��������Ƶװ����ӡ�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
int main (void)
{
    targetInit();                                                       /*  ��ʼ��Ŀ��壬����ɾ��      */
    pinInit();                                                          /*  ���ų�ʼ��                  */
    uartInit();
    SYSAHBCLKCTRL |= (1ul << 6);                                        /*  ʹ��GPIOģ��ʱ��            */

    iSendStr(0, 0, 0x30, "go !! \r\n");
    // BEEP_INIT();
    // GPIO3DIR &= ~KEY;                                                   /*  ����P3.0Ϊ����              */
    // GPIO3IS   = 0x00;                                                   /*  P3.0Ϊ�����ж�              */
    // GPIO3IEV &= ~KEY;                                                    /*  �������ж�                  */
    // GPIO3IE  |= KEY;                                                    /*  P3.0�жϲ�����              */
    // zyIsrSet(NVIC_PIOINT3, (unsigned long)GPIOIsr, PRIO_ONE);
    
    
    
    // ������P0.2 �±����ж� keydown
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

    // targetInit();                                                       /*  ��ʼ��Ŀ��壬����ɾ��      */
    // pinInit();

    // uartInit();                                                         /*  ���ڳ�ʼ��                  */
    //ADCInit();                                                          /*  ADCģ���ʼ��               */


    //iSendStr(0, 0, 0x30, GcRcvBuf);                                 /*  �����ݷ��͵�������ʾ        */
    // myDelay(2000);
    // }
// }
/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
