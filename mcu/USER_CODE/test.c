#include "..\config.h"
#include "mcu.h"
/*********************************************************************************************************
  �궨��
*********************************************************************************************************/
char     GcRcvBuf[20] = {0};                                                  /*  AD�ɼ���������              */

void penDetectTest()
{
    SET_GPIO_syb_n_bit_0(DIR,3,1)
    if(IS_GPIO_syb_n_bit_1(DATA,3,1))
    {
        LED_ON(2,1)
    }
    else if(IS_GPIO_syb_n_bit_0(DATA,3,1))
    {
        LED_OFF(2,1)
        //myDelay(5000);
    }
}


int main (void)
{   
    targetInit();                                                       /*  ��ʼ��Ŀ��壬����ɾ��      */
    pinInit();                                                          /*  ���ų�ʼ��                  */
    uartInit();
    SYSAHBCLKCTRL |= (1ul << 6);                                        /*  ʹ��GPIOģ��ʱ��            */
    
    walTopInit();                                                       /*  ��������ʼ��                  */

    SET_GPIO_syb_n_bit_0(DIR,3,1)                                                   /*  ����P3.0Ϊ����              */
    SET_GPIO_syb_n_bit_value(IS,3,1)                                                   /*  P3.0Ϊ�����ж�              */
    SET_GPIO_syb_n_bit_0(IBE,3,1)                                                    /*  �������ж�                  */
    SET_GPIO_syb_n_bit_1(IE,3,1)                                                      /*  P3.0�жϲ�����              */
    
    zyIsrSet(NVIC_PIOINT3, (unsigned long)penDetectTest, PRIO_ONE);
    //zyIsrSet(NVIC_PIOINT3, (unsigned long)penDetectTest, PRIO_ONE);
   while (1) ;
}

/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
