/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               zy_if.h
** Latest modified Date:    2009-07-23
** Latest Version:          1.00
** Descriptions:            �����ӿں���,��������Ȩģʽ����
**
**--------------------------------------------------------------------------------------------------------
** Created by:              Chenmingji
** Created date:            2009-07-23
** Version:                 1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/
#ifndef __ZY_IF_H
#define __ZY_IF_H

#ifdef __cplusplus
extern "C" {
#endif                                                                  /*  __cplusplus                 */

/*********************************************************************************************************
  ����ֵ����
*********************************************************************************************************/
#define ZY_OK                0                                          /*  �����ɹ�                    */
#define ZY_NOT_OK            1                                          /*  ����ʧ��                    */
#define ZY_PARAMETER_ERR     2                                          /*  ��������                    */
#define ZY_NO_FIND_FUNCTION  6                                          /*  û�з���ָ������            */
#define ZY_NO_MEMORY         12                                         /*  �ڴ治��                    */
#define ZY_TIME_OUT          13                                         /*  ��ʱ                        */
#define ZY_NO_FIND_OBJECT    16                                         /*  û�з��ֶ���                */

/*********************************************************************************************************
  64λ������ض���
*********************************************************************************************************/
typedef long long   INT64S;                                             /*  64λ�з�����                */

#define zyLlAdd(a, b, c)     a = (INT64S)(b) + (INT64S)(c)              /*  64λ�з��żӷ�a=b+c         */
#define zyLlSub(a, b, c)     a = (INT64S)(b) - (INT64S)(c)              /*  64λ�з��ż���a=b-c         */
#define zyLlMul(a, b, c)     a = (INT64S)(b) * (INT64S)(c)              /*  64λ�з��ų˷�a=b*c         */
#define zyLlDiv(a, b, c)     a = (INT64S)(b) / (INT64S)(c)              /*  64λ�з��ų���a=b/c         */
#define zyLlMod(a, b, c)     a = (INT64S)(b) % (INT64S)(c)              /*  64λ�з��ų���a=b/c         */
#define zyLlSet(a, b)        a = (INT64S)(b)                            /*  64λ�з��Ÿ�ֵ              */
#define zyLlIsLess(a, b)     ((INT64S)(a) < (INT64S)(b))                /*  64λ�з���С���ж�          */
#define zyLlSet32(a, b)      a = (INT32U)(b)                            /*  64λ�з��Ÿ�ֵ��32λ�޷�����*/
#define zyLlSet32s(a, b)     a = (INT32S)(b)                            /*  64λ�з��Ÿ�ֵ��32λ�з�����*/

/*********************************************************************************************************
  ��λģʽ
*********************************************************************************************************/
#define ZY_POWER_RESET       0                                          /*  �ϵ縴λ                    */
#define ZY_HARD_RESET        1                                          /*  Ӳ����λ                    */
#define ZY_SOFT_RESET        2                                          /*  �����λ                    */

/*********************************************************************************************************
** Function name:           zyIfInit
** Descriptions:            �ӿڳ�ʼ��
** input parameters:        none
** output parameters:       none
** Returned value:          ZY_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
*********************************************************************************************************/
extern INT32S zyIfInit(void);

/*********************************************************************************************************
** Function name:           zyReset
** Descriptions:            ϵͳ��λ
** input parameters:        uiMode: ZY_POWER_RESET: �ϵ縴λ
**                                  ZY_HARD_RESET:  Ӳ����λ
**                                  ZY_SOFT_RESET:  �����λ
**                                  ����:           ��ϵͳ���
** output parameters:       none
** Returned value:          none
*********************************************************************************************************/
extern void zyReset(unsigned int uiMode);

/*********************************************************************************************************
** Function name:           zyIrqDisable
** Descriptions:            ��ֹ�ж�
** input parameters:        none
** output parameters:       none
** Returned value:          ZY_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
*********************************************************************************************************/
extern __asm INT32S zyIrqDisable(void);

/*********************************************************************************************************
** Function name:           zyIrqEnable
** Descriptions:            �����ж�
** input parameters:        none
** output parameters:       none
** Returned value:          ZY_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
*********************************************************************************************************/
extern __asm INT32S zyIrqEnable(void);

/*********************************************************************************************************
** Function name:           zyIsrSet
** Descriptions:            �����жϷ������
** input parameters:        uiChannel:  �ж�ͨ����
**                          ulFunction: �жϷ�������ַ
**                          uiPrio:     �ж����ȼ�
** output parameters:       none
** Returned value:          zy_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
*********************************************************************************************************/
extern INT32S zyIsrSet(unsigned int uiChannel, unsigned long ulFunction, unsigned int uiPrio);

/*********************************************************************************************************
** Function name:           zyIsrClr
** Descriptions:            ����жϷ������
** input parameters:        uiChannel:  �ж�ͨ����
** output parameters:       none
** Returned value:          ZY_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
*********************************************************************************************************/
extern INT32S zyIsrClr(unsigned int uiChannel);

/*********************************************************************************************************
** Function name:           zyIsrDisable
** Descriptions:            ��ָֹ���ж�
** input parameters:        uiChannel:  �ж�ͨ����
** output parameters:       none
** Returned value:          zy_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
*********************************************************************************************************/
extern INT32S zyIsrDisable(unsigned int uiChannel);

/*********************************************************************************************************
** Function name:           zyIsrEnable
** Descriptions:            ����ָ���ж�
** input parameters:        uiChannel:  �ж�ͨ����
** output parameters:       none
** Returned value:          ZY_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
*********************************************************************************************************/
extern INT32S zyIsrEnable(unsigned int uiChannel);

/*********************************************************************************************************
** Function name:           zyHeapMalloc
** Descriptions:            �ѷ����ڴ�
** input parameters:        ulSize: �ڴ��С
** output parameters:       none
** Returned value:          �ڴ��ַ,NULLΪ���ɹ�
*********************************************************************************************************/
extern void *zyHeapMalloc(INT32U ulSize);

/*********************************************************************************************************
** Function name:           zyHeapFree
** Descriptions:            ���ͷ��ڴ�
** input parameters:        pvPrt: Ҫ�ͷŵ��ڴ�
** output parameters:       none
** Returned value:          ZY_OK: �ɹ�
**                          ����:  ����,����ֵ�ο�zy_if.h
*********************************************************************************************************/
extern INT32S zyHeapFree(void *pvPrt);

#ifdef __cplusplus
}
#endif                                                                  /*  __cplusplus                 */

#endif                                                                  /*  __ZY_IF_H                   */

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
