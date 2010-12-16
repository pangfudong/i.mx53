#include "mcu.h"
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
** Function name:       myDelay
** Descriptions:        LED��˸
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        ���ڳ�ʼ��������Ϊ8λ����λ��1λ��ʼλ��1λֹͣλ������żУ�飬������Ϊ19200
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
    usFdiv = (FAHBCLK / UARTCLKDIV / 16) / 19200;                       /*  ���ò�����                  */
    U0DLM  = usFdiv / 256;
    U0DLL  = usFdiv % 256; 
    U0LCR  = 0x03;                                                      /*  ����������                  */
    U0FCR  = 0x07;
}

/*********************************************************************************************************
** Function name:       uartRecvByte
** Descriptions:        �Ӵ��ڽ����ӽ����ݣ����ȴ����ݽ�����ɣ�ʹ�ò�ѯ��ʽ
** input parameters:    ucDat:   Ҫ���յ�����
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void uartRecvByte (INT8U *ucDat)
{
    *ucDat = U0RBR;                                                      /*  ��������                    */
    while ((U0LSR & 0x40) == 0){                                        /*  �ȴ����ݽ������            */
	}
}

/*********************************************************************************************************
** Function name:       uartRecvStr
** Descriptions:        �򴮿ڽ����ַ���
** input parameters:    puiStr:   Ҫ���ܵ��ַ���ָ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void uartRecvStr (INT8U *pucStr)
{
    while (1){
        if (*pucStr == '\0') {
		    break;                                                      /*  �������������˳�            */
		}
        uartRecvByte (pucStr++);
    }
}

/*********************************************************************************************************
** Function name:       sleep
** Descriptions:        �ô�����sleep������ߵ�ƽ
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void sleep (void)
{
    SET_GPIO_syb_n_bit_1(DIR,0,0)                     /*  �������                    */
    SET_GPIO_syb_n_bit_1(DATA,0,0)                    /*  ����ߵ�ƽ                    */
}

/*********************************************************************************************************
** Function name:       active
** Descriptions:        �ô�����active������͵�ƽ
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void active (void)
{
    SET_GPIO_syb_n_bit_1(DIR,0,1)                     /*  �������                    */
    SET_GPIO_syb_n_bit_0(DATA,0,1)                    /*  ����͵�ƽ                    */
}

/*********************************************************************************************************
** Function name:       reset
** Descriptions:        reset����������Ҫ���ϵ�֮��������͵�ƽ100ms��������ߵ�ƽ
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void reset (void)
{
    SET_GPIO_syb_n_bit_1(DIR,2,6)                     /*  �������                    */
    SET_GPIO_syb_n_bit_0(DATA,2,6)                    /*  ����͵�ƽ                    */
    myDelay(100);                                     /*  ����100ms                    */
    SET_GPIO_syb_n_bit_1(DATA,2,6)                    /*  ����ߵ�ƽ                    */
}

/*********************************************************************************************************
** Function name:       walTopInit
** Descriptions:        walTopInit����������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void walTopInit (void)
{
    active();
    reset();
}
