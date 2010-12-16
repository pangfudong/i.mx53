#include "..\config.h"
#include "mcu.h"
/*********************************************************************************************************
  宏定义
*********************************************************************************************************/
char     GcRcvBuf[20] = {0};                                                  /*  AD采集到的数据              */

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
    targetInit();                                                       /*  初始化目标板，切勿删除      */
    pinInit();                                                          /*  引脚初始化                  */
    uartInit();
    SYSAHBCLKCTRL |= (1ul << 6);                                        /*  使能GPIO模块时钟            */
    
    walTopInit();                                                       /*  触摸屏初始化                  */

    SET_GPIO_syb_n_bit_0(DIR,3,1)                                                   /*  设置P3.0为输入              */
    SET_GPIO_syb_n_bit_value(IS,3,1)                                                   /*  P3.0为边沿中断              */
    SET_GPIO_syb_n_bit_0(IBE,3,1)                                                    /*  上下沿中断                  */
    SET_GPIO_syb_n_bit_1(IE,3,1)                                                      /*  P3.0中断不屏蔽              */
    
    zyIsrSet(NVIC_PIOINT3, (unsigned long)penDetectTest, PRIO_ONE);
    //zyIsrSet(NVIC_PIOINT3, (unsigned long)penDetectTest, PRIO_ONE);
   while (1) ;
}

/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
