/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               zy_if.h
** Latest modified Date:    2009-07-23
** Latest Version:          1.00
** Descriptions:            环境接口函数,必须在特权模式运行
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
  返回值定义
*********************************************************************************************************/
#define ZY_OK                0                                          /*  操作成功                    */
#define ZY_NOT_OK            1                                          /*  操作失败                    */
#define ZY_PARAMETER_ERR     2                                          /*  参数错误                    */
#define ZY_NO_FIND_FUNCTION  6                                          /*  没有发现指定函数            */
#define ZY_NO_MEMORY         12                                         /*  内存不足                    */
#define ZY_TIME_OUT          13                                         /*  超时                        */
#define ZY_NO_FIND_OBJECT    16                                         /*  没有发现对象                */

/*********************************************************************************************************
  64位变量相关定义
*********************************************************************************************************/
typedef long long   INT64S;                                             /*  64位有符号数                */

#define zyLlAdd(a, b, c)     a = (INT64S)(b) + (INT64S)(c)              /*  64位有符号加法a=b+c         */
#define zyLlSub(a, b, c)     a = (INT64S)(b) - (INT64S)(c)              /*  64位有符号减法a=b-c         */
#define zyLlMul(a, b, c)     a = (INT64S)(b) * (INT64S)(c)              /*  64位有符号乘法a=b*c         */
#define zyLlDiv(a, b, c)     a = (INT64S)(b) / (INT64S)(c)              /*  64位有符号除法a=b/c         */
#define zyLlMod(a, b, c)     a = (INT64S)(b) % (INT64S)(c)              /*  64位有符号除法a=b/c         */
#define zyLlSet(a, b)        a = (INT64S)(b)                            /*  64位有符号赋值              */
#define zyLlIsLess(a, b)     ((INT64S)(a) < (INT64S)(b))                /*  64位有符号小于判定          */
#define zyLlSet32(a, b)      a = (INT32U)(b)                            /*  64位有符号赋值给32位无符号数*/
#define zyLlSet32s(a, b)     a = (INT32S)(b)                            /*  64位有符号赋值给32位有符号数*/

/*********************************************************************************************************
  复位模式
*********************************************************************************************************/
#define ZY_POWER_RESET       0                                          /*  上电复位                    */
#define ZY_HARD_RESET        1                                          /*  硬件复位                    */
#define ZY_SOFT_RESET        2                                          /*  软件复位                    */

/*********************************************************************************************************
** Function name:           zyIfInit
** Descriptions:            接口初始化
** input parameters:        none
** output parameters:       none
** Returned value:          ZY_OK: 成功
**                          负数:  错误,绝对值参考zy_if.h
*********************************************************************************************************/
extern INT32S zyIfInit(void);

/*********************************************************************************************************
** Function name:           zyReset
** Descriptions:            系统复位
** input parameters:        uiMode: ZY_POWER_RESET: 上电复位
**                                  ZY_HARD_RESET:  硬件复位
**                                  ZY_SOFT_RESET:  软件复位
**                                  其它:           与系统相关
** output parameters:       none
** Returned value:          none
*********************************************************************************************************/
extern void zyReset(unsigned int uiMode);

/*********************************************************************************************************
** Function name:           zyIrqDisable
** Descriptions:            禁止中断
** input parameters:        none
** output parameters:       none
** Returned value:          ZY_OK: 成功
**                          负数:  错误,绝对值参考zy_if.h
*********************************************************************************************************/
extern __asm INT32S zyIrqDisable(void);

/*********************************************************************************************************
** Function name:           zyIrqEnable
** Descriptions:            允许中断
** input parameters:        none
** output parameters:       none
** Returned value:          ZY_OK: 成功
**                          负数:  错误,绝对值参考zy_if.h
*********************************************************************************************************/
extern __asm INT32S zyIrqEnable(void);

/*********************************************************************************************************
** Function name:           zyIsrSet
** Descriptions:            设置中断服务程序
** input parameters:        uiChannel:  中断通道号
**                          ulFunction: 中断服务程序地址
**                          uiPrio:     中断优先级
** output parameters:       none
** Returned value:          zy_OK: 成功
**                          负数:  错误,绝对值参考zy_if.h
*********************************************************************************************************/
extern INT32S zyIsrSet(unsigned int uiChannel, unsigned long ulFunction, unsigned int uiPrio);

/*********************************************************************************************************
** Function name:           zyIsrClr
** Descriptions:            清除中断服务程序
** input parameters:        uiChannel:  中断通道号
** output parameters:       none
** Returned value:          ZY_OK: 成功
**                          负数:  错误,绝对值参考zy_if.h
*********************************************************************************************************/
extern INT32S zyIsrClr(unsigned int uiChannel);

/*********************************************************************************************************
** Function name:           zyIsrDisable
** Descriptions:            禁止指定中断
** input parameters:        uiChannel:  中断通道号
** output parameters:       none
** Returned value:          zy_OK: 成功
**                          负数:  错误,绝对值参考zy_if.h
*********************************************************************************************************/
extern INT32S zyIsrDisable(unsigned int uiChannel);

/*********************************************************************************************************
** Function name:           zyIsrEnable
** Descriptions:            允许指定中断
** input parameters:        uiChannel:  中断通道号
** output parameters:       none
** Returned value:          ZY_OK: 成功
**                          负数:  错误,绝对值参考zy_if.h
*********************************************************************************************************/
extern INT32S zyIsrEnable(unsigned int uiChannel);

/*********************************************************************************************************
** Function name:           zyHeapMalloc
** Descriptions:            堆分配内存
** input parameters:        ulSize: 内存大小
** output parameters:       none
** Returned value:          内存地址,NULL为不成功
*********************************************************************************************************/
extern void *zyHeapMalloc(INT32U ulSize);

/*********************************************************************************************************
** Function name:           zyHeapFree
** Descriptions:            堆释放内存
** input parameters:        pvPrt: 要释放的内存
** output parameters:       none
** Returned value:          ZY_OK: 成功
**                          负数:  错误,绝对值参考zy_if.h
*********************************************************************************************************/
extern INT32S zyHeapFree(void *pvPrt);

#ifdef __cplusplus
}
#endif                                                                  /*  __cplusplus                 */

#endif                                                                  /*  __ZY_IF_H                   */

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
