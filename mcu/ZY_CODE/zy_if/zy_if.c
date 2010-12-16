/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               zy_if.c
** Latest modified Date:    2009-07-23
** Latest Version:          V1.01
** Descriptions:            �û���д�Ļ����ӿں���,��������Ȩģʽ����
**
**--------------------------------------------------------------------------------------------------------
** Created by:              Chenmingji
** Created date:            2009-07-23
** Version:                 V1.00
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             Lanwuqiang
** Modified date:           2010-01-26
** Version:                 V1.01
** Descriptions:            ����CM0�жϵ����ú���
**
*********************************************************************************************************/
#include "..\..\config.h"
#include ".\zy_if.h"
#include "..\..\cfg_file\zy_if\zy_if_cfg.h"
#include <string.h>

/*********************************************************************************************************
** Function name:           zyIfInit
** Descriptions:            �ӿڳ�ʼ��
** input parameters:        none
** output parameters:       none
** Returned value:          zy_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
** Created by:              Chenmingji
** Created Date:            2009-07-23
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
*********************************************************************************************************/
INT32S zyIfInit (void)
{
    return ZY_OK;
}

/*********************************************************************************************************
** Function name:           zyReset
** Descriptions:            ϵͳ��λ
** input parameters:        uiMode: ZY_POWER_RESET: �ϵ縴λ
**                                  ZY_HARD_RESET:  Ӳ����λ
**                                  ZY_SOFT_RESET:  �����λ
**                                  ����:           ��ϵͳ���
** output parameters:       none
** Returned value:          none
** Created by:              Chenmingji
** Created Date:            2009-07-23
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
*********************************************************************************************************/
void zyReset (unsigned int uiMode)
{
    switch (uiMode) {

    case ZY_POWER_RESET:                                                /*  ��ϵͳ�ϵ縴λ��ͬӲ����λ  */

#if 0
        break;
#endif                                                                  /*  __0                         */

    case ZY_HARD_RESET:
        AITCR = (0x05fa << 16) + 4;
        break;
    
    default:                                                            /*  ��������ȷ����λ            */
        break;
    }
}

/*********************************************************************************************************
** Function name:           zyHeapMalloc
** Descriptions:            �ѷ����ڴ�
** input parameters:        ulSize: �ڴ��С
** output parameters:       none
** Returned value:          �ڴ��ַ,NULLΪ���ɹ�
** Created by:              Chenmingji
** Created Date:            2009-07-23
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
*********************************************************************************************************/
void *zyHeapMalloc (INT32U ulSize)
{
    void *pvRt = NULL;                                                  /*  ����ֵ                      */

    zyIrqDisable();
    pvRt = malloc((size_t)ulSize);
    zyIrqEnable();
    return pvRt;
}

/*********************************************************************************************************
** Function name:           zyHeapFree
** Descriptions:            ���ͷ��ڴ�
** input parameters:        pvPrt: Ҫ�ͷŵ��ڴ�
** output parameters:       none
** Returned value:          zy_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
** Created by:              Chenmingji
** Created Date:            2009-07-23
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
*********************************************************************************************************/
INT32S zyHeapFree (void *pvPrt)
{
    zyIrqDisable();
    free(pvPrt);
    zyIrqEnable();
    return ZY_OK;
}

/*********************************************************************************************************
** Function name:           zyIrqDisable
** Descriptions:            ��ֹ�ж�
** input parameters:        none
** output parameters:       none
** Returned value:          zy_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
** Created by:              Chenmingji
** Created Date:            2009-07-23
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
*********************************************************************************************************/
__asm INT32S zyIrqDisable (void)
{
    CPSID   I
    MOVS    R0, #0
    BX      LR
}

/*********************************************************************************************************
** Function name:           zyIrqEnable
** Descriptions:            �����ж�
** input parameters:        none
** output parameters:       none
** Returned value:          zy_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�.h
** Created by:              Chenmingji
** Created Date:            2009-07-23
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
*********************************************************************************************************/
__asm INT32S zyIrqEnable (void)
{
    CPSIE   I
    MOVS    R0, #0
    BX      LR
}

/*********************************************************************************************************
** Function name:           zyIsrSet
** Descriptions:            IOϵͳ�����жϷ������
** input parameters:        uiChannel:  �ж�ͨ����
**                          ulFunction: �жϷ�������ַ
**                          uiPrio:     �ж����ȼ�
** output parameters:       none
** Returned value:          zy_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
** Created by:              Chenmingji
** Created Date:            2009-07-23
**--------------------------------------------------------------------------------------------------------
** Modified by:             Lanwuqiang
** Modified date:           2010-02-04
*********************************************************************************************************/
INT32S zyIsrSet (unsigned int uiChannel, unsigned long ulFunction, unsigned int uiPrio)
{
    unsigned int uiTmp1, uiTmp2, uiTmp3, uiTmp4, uiTmp5;
    
    if (uiChannel > MAX_VICS) {
        return -ZY_NOT_OK;
    }

    zyIrqDisable();
                  
    if (uiChannel >= 16) {
        uiTmp3 = uiChannel - 16;
        uiTmp1 = uiTmp3 / 32;
        uiTmp2 = uiTmp3 % 32;
        uiTmp4 = uiTmp3 >> 2;                                           /*  ֻ����Ѱַ      div4        */
        uiTmp5 = (uiTmp3 & 0x03) * 8;                                   /*  ���ȼ��ƶ�λ��  mod 4       */

        ((INT32U *)0xE000E100)[uiTmp1]  = 1ul << uiTmp2;
        ((INT32U *)0xE000E400)[uiTmp4] &= (~(0xFF << uiTmp5));
        ((INT32U *)0xE000E400)[uiTmp4] |= (uiPrio << uiTmp5);

    } else {
      switch (uiChannel) {
       
        case MMI:
            SHCSR = SHCSR | (1 << 16);
            break;

        case BFI:
            SHCSR = SHCSR | (1 << 17);
            break;

        case UFI:
            SHCSR = SHCSR | (1 << 18);
            break;
              
        default:
            break;
        }
        if (uiChannel >= MMI) {
            uiTmp3 = uiChannel - MMI;
            uiTmp4 = uiTmp3 >> 2;                                       /*  ֻ����Ѱַ      div4        */
            uiTmp5 = (uiTmp3 & 0x03) * 8;                               /*  ���ȼ�λ��      mod 4       */
            ((INT32U *)0xE000ED18)[uiTmp4]  &= (~(0xFF << uiTmp5));
            ((INT32U *)0xE000ED18)[uiTmp4]  |= uiPrio << uiTmp5;
        }
    }

#if VECTOR_TABLE_IN_FLASH == 0
    ((unsigned long *)VTOR)[uiChannel] = ulFunction;
#endif                                                                  /* __VECTOR_TABLE_IN_FLASH      */
    zyIrqEnable();
    return ZY_OK;
}

/*********************************************************************************************************
** Function name:           zyIsrClr
** Descriptions:            IOϵͳ����жϷ������
** input parameters:        uiChannel:  �ж�ͨ����
** output parameters:       none
** Returned value:          zy_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
** Created by:              Chenmingji
** Created Date:            2009-07-23
**--------------------------------------------------------------------------------------------------------
** Modified by:             Lanwuqiang
** Modified date:           2010-02-04
*********************************************************************************************************/
INT32S zyIsrClr (unsigned int uiChannel)
{
    unsigned int uiTmp1, uiTmp2, uiTmp3, uiTmp4, uiTmp5;
    
    if (uiChannel > MAX_VICS) {
        return -ZY_NOT_OK;
    }

    zyIrqDisable();
    
    if (uiChannel >= 16) {
        uiTmp3 = uiChannel - 16;
        uiTmp1 = uiTmp3 / 32;
        uiTmp2 = uiTmp3 % 32;
        uiTmp4 = uiTmp3 >> 2;                                           /*  ֻ����Ѱַ         div4     */
        uiTmp5 = (uiTmp3 & 0x03) * 8;                                   /*  ���ȼ���������λ�� mod 4    */

        ((INT32U *)0xE000E180)[uiTmp1]   = 1ul << uiTmp2;
        ((INT32U *)0xE000E400)[uiTmp4]  &= (~(0xFF << uiTmp5));

    } else {
       
        switch (uiChannel) {
        
        case MMI:
            SHCSR = SHCSR & ~(1 << 16);
            break;

        case BFI:
            SHCSR = SHCSR & ~(1 << 17);
            break;

        case UFI:
            SHCSR = SHCSR & ~(1 << 18);
            break;

        default:
            break;
        }
        if (uiChannel >= MMI) {
            uiTmp3 = uiChannel - MMI;
            uiTmp4 = uiTmp3 >> 2;                                       /*  ֻ����Ѱַ      div4        */
            uiTmp5 = (uiTmp3 & 0x03) * 8;                               /*  ���ȼ�����λ��  mod 4       */
            ((INT32U *)0xE000ED18)[uiTmp4]  = (~(0xFF << uiTmp5));
        }
    }

#if VECTOR_TABLE_IN_FLASH == 0
    ((unsigned long *)VTOR)[uiChannel] = 0;
#endif                                                                  /* __VECTOR_TABLE_IN_FLASH      */
    zyIrqEnable();
    return ZY_OK;
}

/*********************************************************************************************************
** Function name:           zyIsrDisable
** Descriptions:            ��ָֹ���ж�
** input parameters:        uiChannel:  �ж�ͨ����
** output parameters:       none
** Returned value:          zy_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
** Created by:              Chenmingji
** Created Date:            2009-07-23
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
*********************************************************************************************************/
INT32S zyIsrDisable (unsigned int uiChannel)
{
    unsigned int uiTmp1, uiTmp2, uiTmp3;
    
    if (uiChannel > MAX_VICS) {
        return -ZY_NOT_OK;
    }
    
    if (uiChannel < 16) {
        return -ZY_NOT_OK;
    }

    zyIrqDisable();
    
    if (uiChannel >= 16) {
        uiTmp3 = uiChannel - 16;
        uiTmp1 = uiTmp3 / 32;
        uiTmp2 = uiTmp3 % 32;
    
        ((INT32U *)0xE000E180)[uiTmp1] = 1ul << uiTmp2;

    } else {

        switch (uiChannel) {
        
        case MMI:
            SHCSR = SHCSR & ~(1 << 16);
            break;

        case BFI:
            SHCSR = SHCSR & ~(1 << 17);
            break;

        case UFI:
            SHCSR = SHCSR & ~(1 << 18);
            break;

        default:
            break;
        }
    }
    zyIrqEnable();
    return ZY_OK;
}

/*********************************************************************************************************
** Function name:           zyIsrEnable
** Descriptions:            ����ָ���ж�
** input parameters:        uiChannel:  �ж�ͨ����
** output parameters:       none
** Returned value:          ZY_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
** Created by:              Chenmingji
** Created Date:            2009-07-23
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
*********************************************************************************************************/
INT32S zyIsrEnable (unsigned int uiChannel)
{
    unsigned int uiTmp1, uiTmp2, uiTmp3;
    
    if (uiChannel > MAX_VICS) {
        return -ZY_NOT_OK;
    }

    zyIrqDisable();
    
    if (uiChannel >= 16) {
        uiTmp3 = uiChannel - 16;
        uiTmp1 = uiTmp3 / 32;
        uiTmp2 = uiTmp3 % 32;
        ((INT32U *)0xE000E100)[uiTmp1] = 1ul << uiTmp2;

    } else {

        switch (uiChannel) {
        
        case MMI:
            SHCSR = SHCSR | (1 << 16);
            break;

        case BFI:
            SHCSR = SHCSR | (1 << 17);
            break;

        case UFI:
            SHCSR = SHCSR | (1 << 18);
            break;

        default:
            break;
        }
    }
    zyIrqEnable();
    return ZY_OK;
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
