/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               rt_sys_c.c
** Latest modified Date:    2009-06-01
** Latest Version:          1.0
** Descriptions:            Ŀ��岿��ʵʱ�⺯������
**
**--------------------------------------------------------------------------------------------------------
** Created by:              Chenmingji
** Created date:            2009-06-01
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
#include "..\..\config.h"
#include <stdlib.h>

/*********************************************************************************************************
  ����ΪһЩ��ϵͳ��صĿ⺯����ʵ��
  ����������ο���������⺯���ֲ�
  �û����Ը����Լ���Ҫ���޸�
*********************************************************************************************************/
/*********************************************************************************************************
  The implementations for some library functions
  For more details, please refer to the ADS compiler handbook and The library
  function manual
  User could change it as needed
*********************************************************************************************************/

#include <rt_sys.h>
#include <rt_misc.h>
#include <time.h>
#include <stdio.h>

#pragma  import(__use_no_semihosting_swi)

/*********************************************************************************************************
  ������������غ���
*********************************************************************************************************/

/*********************************************************************************************************
** Function name:           _sys_exit
** Descriptions:            ���س�ʼ��ջ�Ͷѵ�λ�ã�һ���û��ʵ��
** input parameters:        �ֲ�δ����ϸ˵��
** output parameters:       none
** Returned value:          r0: �ѻ�ַ
**                          r1: ��ջ��ַ������ջ���е���ߵ�ַ
**                          r2: ������
**                          r3: ��ջ���ƣ�����ջ���е���͵�ַ
*********************************************************************************************************/
#if 0
__value_in_regs struct __initial_stackheap __user_initial_stackheap (unsigned R0,
                                                                     unsigned SP,
                                                                     unsigned R2)
{
}
#endif                                                                  /*  0                           */

/*********************************************************************************************************
** Function name:           _sys_exit
** Descriptions:            ���˳����������дӿ��е��˳����ն������_sys_exit()��
** input parameters:        iReturnCode: �˳�����
** output parameters:       none
** Returned value:          none
*********************************************************************************************************/
void _sys_exit (int iReturnCode)
{
    while (1) {
    }
}

/*********************************************************************************************************
** Function name:           _ttywrch
** Descriptions:            �˺�����һ���ַ�д�뵽����̨�С� ����̨�����ѱ��ض��� �����򲻵��ѣ�
**                          ����Ҫ���˺����������������̡�
** input parameters:        iCh: ������ַ���
** output parameters:       none
** Returned value:          none
*********************************************************************************************************/
void _ttywrch (int iCh)
{
}

/*********************************************************************************************************
** Function name:           _sys_open
** Descriptions:            �˺�����һ���ļ���
** input parameters:        pcFileName: �ļ���
**                          iOpenMode:  ��ģʽ,��һ��λӳ��,��λͨ��ֱ�Ӷ�Ӧ��ISOģʽ�淶.
**                                      ��ϸ��Ϣ�����rt_sys.h
** output parameters:       none
** Returned value:          �ļ����
*********************************************************************************************************/
FILEHANDLE _sys_open (const char *pcFileName, int iOpenMode)
{
    return -1;
}

/*********************************************************************************************************
** Function name:           _sys_close
** Descriptions:            �˺����ر���ǰʹ�� _sys_open() �򿪵��ļ�
** input parameters:        fhHandle: �ļ����
** output parameters:       none
** Returned value:          �ļ����
*********************************************************************************************************/
int _sys_close (FILEHANDLE fhHandle)
{
    return -1;
}

/*********************************************************************************************************
** Function name:           _sys_read
** Descriptions:            �˺������ļ����ݶ�ȡ����������
** input parameters:        fhHandle: �ļ����
**                          iLen:     Ҫ����������Ŀ
**                          iMode:    û��ʹ��
** output parameters:       puBcuf:   ����������
** Returned value:          δ����������Ŀ
*********************************************************************************************************/
int _sys_read (FILEHANDLE fhHandle, unsigned char *puBcuf, unsigned int iLen, int iMode)
{
    return (int)0x80000000;
}

/*********************************************************************************************************
** Function name:           _sys_write
** Descriptions:            ������������д�뵽��ǰʹ�� _sys_open() �򿪵��ļ���
** input parameters:        fhHandle: �ļ����
**                          puBcuf:   Ҫд������
**                          iLen:     Ҫд��������Ŀ
**                          iMode:    û��ʹ��
** output parameters:       none
** Returned value:          δд��������Ŀ
*********************************************************************************************************/
int _sys_write (FILEHANDLE fhHandle, const unsigned char *puBcuf, unsigned int iLen, int iMode)
{
    return 0x80000000;
}

/*********************************************************************************************************
** Function name:           _sys_ensure
** Descriptions:            _sys_ensure() ���ý�ˢ�����ļ���� fh ��������κλ�����������ȷ��������
**                          �洢�����ϵ��ļ���������״̬��
** input parameters:        fhHandle: �ļ����
** output parameters:       none
** Returned value:          ������ִ�������Ϊ������
*********************************************************************************************************/
int _sys_ensure (FILEHANDLE fhHandle)
{
    return 0;
}

/*********************************************************************************************************
** Function name:           _sys_flen
** Descriptions:            �˺��������ļ��ĵ�ǰ���ȡ�
**                          �������ϵͳ����������ļ�ĩβ��������������Զ��� fseek()��
**                          �����Ͳ�����Ҫʹ�� _sys_flen() �ˡ�
** input parameters:        fhHandle: �ļ����
** output parameters:       none
** Returned value:          ������ִ�������Ϊ������
*********************************************************************************************************/
long _sys_flen (FILEHANDLE fhHandle)
{
    return -1;
}

/*********************************************************************************************************
** Function name:           _sys_seek
** Descriptions:            �˺������ļ�ָ��������ļ���ͷ֮���ƫ��ΪlPos��λ��
** input parameters:        fhHandle: �ļ����
**                          lPos:     ƫ����
** output parameters:       none
** Returned value:          ���û�г��ִ��󣬽��Ϊ�Ǹ�����������ִ��󣬽��Ϊ������
*********************************************************************************************************/
int _sys_seek (FILEHANDLE fhHandle, long lPos)
{
    return -1;
}

/*********************************************************************************************************
** Function name:           _sys_istty
** Descriptions:            �˺���ȷ���ļ�����Ƿ�ָ��һ���նˡ�
** input parameters:        fhHandle: �ļ����
** output parameters:       none
** Returned value:          0:    û�н����豸
**                          1:    �н����豸
**                          ����: ���ִ���
*********************************************************************************************************/
int _sys_istty (FILEHANDLE fhHandle)
{
    return -1;
}

/*********************************************************************************************************
** Function name:           _sys_tmpnam
** Descriptions:            �˺�������ʱ�ļ����ļ���� fileno ת��ΪΨһ���ļ���
** input parameters:        iFileNo:  �ļ����
**                          uiMaxLen: �ļ���������
** output parameters:       pcName:   �ļ��� 
** Returned value:          ����û����ȷ˵��
*********************************************************************************************************/
int _sys_tmpnam (char * pcName, int iFileNo, unsigned int uiMaxLen)
{
    return 0;
}

/*********************************************************************************************************
** Function name:           _sys_command_string
** Descriptions:            �˺����ӵ��õ�ǰӦ�ó���Ļ����м������ڵ��ø�Ӧ�ó���������С�(δʵ��)
** input parameters:        pcCmd: �������
**                          iLen:  �����������
** output parameters:       none   
** Returned value:          ����ɹ����򷵻�һ��ָ�������е�ָ�롣
**                          ���ʧ�ܣ��򷵻� NULL��
*********************************************************************************************************/
char *_sys_command_string (char *pcCmd, int iLen)
{
    return NULL;
}

/*********************************************************************************************************
** Function name:           clock
** Descriptions:            ��õ�ǰ����ʱ��
** input parameters:        none
** output parameters:       none   
** Returned value:          ��(1/__CLK_TCK)��Ϊ��λ��ִ��ʱ�䡣
*********************************************************************************************************/
clock_t clock(void)
{
    return 0;
}

/*********************************************************************************************************
** Function name:           _clock_init
** Descriptions:            ����һ������ clock() �Ŀ�ѡ��ʼ������
** input parameters:        none
** output parameters:       none   
** Returned value:          none
*********************************************************************************************************/
#if 0
void _clock_init(void)
{
}
#endif                                                                  /*  0                           */

/*********************************************************************************************************
** Function name:           time
** Descriptions:            ���� time.h �еı�׼ C �� time() ������(δ��ȫʵ��)
** input parameters:        none
** output parameters:       none   
** Returned value:          ����ֵ�ǵ�ǰ����ʱ��Ľ���ֵ��
**                          �������ʱ�䲻���ã��򷵻�ֵ (time_t)-1�� ��� ptTimer ���� NULL ָ�룬�򻹻�
**                          ������ֵ����� time_t*��
*********************************************************************************************************/
time_t time (time_t *ptTimer)
{
    time_t      tRt;                                                    /*  ����ֵ                      */
    struct tm   tmDataTime;                                             /*  ��ǰ����ʱ��                */
    
    /*
     * ��ӻ�õ�ǰ����ʱ��Ĵ���
     * struct tm��������
     * struct tm {
     *   int tm_sec;   // seconds after the minute, 0 to 60
     *                    (0 - 60 allows for the occasional leap second)
     *   int tm_min;   // minutes after the hour, 0 to 59
     *   int tm_hour;  // hours since midnight, 0 to 23
     *   int tm_mday;  // day of the month, 1 to 31
     *   int tm_mon;   // months since January, 0 to 11
     *   int tm_year;  // years since 1900
     *   int tm_wday;  // days since Sunday, 0 to 6
     *   int tm_yday;  // days since January 1, 0 to 365
     *   int tm_isdst; // Daylight Savings Time flag
     * };
     */
    
    tRt = mktime(&tmDataTime);
    
    if (ptTimer != NULL) {
        *ptTimer = tRt;
    }
    return tRt;
}

/*********************************************************************************************************
** Function name:           remove
** Descriptions:            ɾ��һ���ļ�
** input parameters:        pcFileName: �ļ���
** output parameters:       none
** Returned value:          ��������ɹ����򷵻� 0�����ʧ�ܣ��򷵻ط���ֵ��
*********************************************************************************************************/
int remove (const char * pcFileName)
{
    return -1;
}

/*********************************************************************************************************
** Function name:           rename
** Descriptions:            �ļ�����
** input parameters:        pcOldName:    ���ļ���
**                          pcNewName:    ���ļ���
** output parameters:       none
** Returned value:          ��������ɹ����򷵻� 0�����ʧ�ܣ��򷵻ط���ֵ�� ����������ط���ֵ����
**                          �Ҹ��ļ���ǰ���ڣ�����ļ���ʹ����ԭʼ���ơ�
*********************************************************************************************************/
int rename (const char *pcOldName, const char *pcNewName)
{
    return -1;
}

/*********************************************************************************************************
** Function name:           system
** Descriptions:            �� pcString ��ָ����ַ������ݵ����������У��������������ʵ�ֶ�
**                          ��ķ�ʽִ�С� string ����ʹ�ÿ�ָ���Բ�����������Ƿ���ڡ�
** input parameters:        pcString: �ַ���
** output parameters:       none
** Returned value:          ����Ա����ǿ�ָ�룬����������������ʱ����ϵͳ�����Ż᷵�ط���ֵ��
**                          ����Ա������ǿ�ָ�룬system() ������������ʵ�ֶ����ֵ��
*********************************************************************************************************/
int system (const char *pcString)
{
    return 0;
}

/*********************************************************************************************************
  End File
*********************************************************************************************************/
