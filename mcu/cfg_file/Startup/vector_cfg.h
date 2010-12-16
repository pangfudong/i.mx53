/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               vic_cfg.h
** Latest modified Date:    2008-12-22
** Latest Version:          1.0
** Descriptions:            异常处理程序定义
**
**--------------------------------------------------------------------------------------------------------
** Created by:              Chenmingji
** Created date:            2008-12-22
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
#ifndef __VIC_CFG_H
#define __VIC_CFG_H

/*********************************************************************************************************
  lpc1000特定异常处理程序定义，VECTOR_TABLE_IN_FLASH = 0时无意义
*********************************************************************************************************/
#define WAKEUP_IRQ_HANDLER              defaultVectorHandle             /*  16~28 Wakeup_IRQ_HANDLER    */

#define SSP1_IRQ_Handle                 defaultVectorHandle             /*  61 SSP                      */
#define I2C_IRQ_Handle                  defaultVectorHandle             /*  56 I2C                      */
#define TIMER16_0_IRQ_Handle            defaultVectorHandle             /*  57 TIMER16_0                */
#define TIMER16_1_IRQ_Handle            defaultVectorHandle             /*  58 TIMER16_1                */
#define TIMER32_0_IRQ_Handle            defaultVectorHandle             /*  59 TIMER32_0                */
#define TIMER32_1_IRQ_Handle            defaultVectorHandle             /*  60 TIMER32_1                */
#define SSP0_IRQ_Handle                 defaultVectorHandle             /*  61 SSP                      */
#define UART_IRQ_Handle                 defaultVectorHandle             /*  62 UART                     */
#define USB_IRQ_Handle                  defaultVectorHandle             /*  63 USB_IRQ                  */
#define USB_FIQ_Handle                  defaultVectorHandle             /*  64 USB_FIQ                  */
#define ADC_IRQ_Handle                  defaultVectorHandle             /*  65 ADC                      */
#define WDT_IRQ_Handle                  defaultVectorHandle             /*  66 WDT                      */
#define BOD_IRQ_Handle                  defaultVectorHandle             /*  67 BOD                      */
#define FMC_IRQ_Handle                  defaultVectorHandle             /*  68 RESERVED                 */
#define PIOINT3_IRQ_Handle              penDetectTest               /*  69 PIOINT3                  */
#define PIOINT2_IRQ_Handle              defaultVectorHandle             /*  70 PIOINT2                  */
#define PIOINT1_IRQ_Handle              defaultVectorHandle             /*  71 PIOINT1                  */
#define PIOINT0_IRQ_Handle              defaultVectorHandle             /*  72 PIOINT0                  */

/*********************************************************************************************************
  异常处理程序定义，VECTOR_TABLE_IN_FLASH = 0时无意义
*********************************************************************************************************/
#define NMI_HANDLE                      defaultVectorHandle
#define HARD_FAULT_HANDLE               defaultVectorHandle
#define SVCALL_HANDLE                   defaultVectorHandle
#define DEBUG_MON_HANDLE                defaultVectorHandle
#define PEND_SV_HANDLE                  defaultVectorHandle
#define SYS_TICK_HANDLE                 defaultVectorHandle

#define IRQ_16_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_17_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_18_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_19_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_20_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_21_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_22_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_23_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_24_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_25_HANDLE                   WAKEUP_IRQ_HANDLER

#define IRQ_26_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_27_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_28_HANDLE                   WAKEUP_IRQ_HANDLER
#define IRQ_29_HANDLE                   defaultVectorHandle

#define IRQ_30_HANDLE                   SSP1_IRQ_Handle
#define IRQ_31_HANDLE                   I2C_IRQ_Handle
#define IRQ_32_HANDLE                   TIMER16_0_IRQ_Handle
#define IRQ_33_HANDLE                   TIMER16_1_IRQ_Handle
#define IRQ_34_HANDLE                   TIMER32_0_IRQ_Handle
#define IRQ_35_HANDLE                   TIMER32_1_IRQ_Handle
#define IRQ_36_HANDLE                   SSP0_IRQ_Handle
#define IRQ_37_HANDLE                   UART_IRQ_Handle
#define IRQ_38_HANDLE                   USB_IRQ_Handle
#define IRQ_39_HANDLE                   USB_FIQ_Handle
#define IRQ_40_HANDLE                   ADC_IRQ_Handle

#define IRQ_41_HANDLE                   WDT_IRQ_Handle
#define IRQ_42_HANDLE                   BOD_IRQ_Handle
#define IRQ_43_HANDLE                   defaultVectorHandle
#define IRQ_44_HANDLE                   PIOINT3_IRQ_Handle
#define IRQ_45_HANDLE                   PIOINT2_IRQ_Handle
#define IRQ_46_HANDLE                   PIOINT1_IRQ_Handle
#define IRQ_47_HANDLE                   PIOINT0_IRQ_Handle

#define IRQ_48_HANDLE                   defaultVectorHandle
#define IRQ_49_HANDLE                   defaultVectorHandle
#define IRQ_50_HANDLE                   defaultVectorHandle
#define IRQ_51_HANDLE                   defaultVectorHandle
#define IRQ_52_HANDLE                   defaultVectorHandle
#define IRQ_53_HANDLE                   defaultVectorHandle
#define IRQ_54_HANDLE                   defaultVectorHandle
#define IRQ_55_HANDLE                   defaultVectorHandle
#define IRQ_56_HANDLE                   defaultVectorHandle
#define IRQ_57_HANDLE                   defaultVectorHandle

#define IRQ_58_HANDLE                   defaultVectorHandle
#define IRQ_59_HANDLE                   defaultVectorHandle
#define IRQ_60_HANDLE                   defaultVectorHandle
#define IRQ_61_HANDLE                   defaultVectorHandle
#define IRQ_62_HANDLE                   defaultVectorHandle
#define IRQ_63_HANDLE                   defaultVectorHandle
#define IRQ_64_HANDLE                   defaultVectorHandle
#define IRQ_65_HANDLE                   defaultVectorHandle
#define IRQ_66_HANDLE                   defaultVectorHandle
#define IRQ_67_HANDLE                   defaultVectorHandle
#define IRQ_68_HANDLE                   defaultVectorHandle
#define IRQ_69_HANDLE                   defaultVectorHandle
#define IRQ_70_HANDLE                   defaultVectorHandle
#define IRQ_71_HANDLE                   defaultVectorHandle
#define IRQ_72_HANDLE                   defaultVectorHandle
#define IRQ_73_HANDLE                   defaultVectorHandle
#define IRQ_74_HANDLE                   defaultVectorHandle
#define IRQ_75_HANDLE                   defaultVectorHandle
#define IRQ_76_HANDLE                   defaultVectorHandle
#define IRQ_77_HANDLE                   defaultVectorHandle
#define IRQ_78_HANDLE                   defaultVectorHandle
#define IRQ_79_HANDLE                   defaultVectorHandle

#define IRQ_80_HANDLE                   defaultVectorHandle
#define IRQ_81_HANDLE                   defaultVectorHandle
#define IRQ_82_HANDLE                   defaultVectorHandle
#define IRQ_83_HANDLE                   defaultVectorHandle
#define IRQ_84_HANDLE                   defaultVectorHandle
#define IRQ_85_HANDLE                   defaultVectorHandle
#define IRQ_86_HANDLE                   defaultVectorHandle
#define IRQ_87_HANDLE                   defaultVectorHandle
#define IRQ_88_HANDLE                   defaultVectorHandle
#define IRQ_89_HANDLE                   defaultVectorHandle

#define IRQ_90_HANDLE                   defaultVectorHandle
#define IRQ_91_HANDLE                   defaultVectorHandle
#define IRQ_92_HANDLE                   defaultVectorHandle
#define IRQ_93_HANDLE                   defaultVectorHandle
#define IRQ_94_HANDLE                   defaultVectorHandle
#define IRQ_95_HANDLE                   defaultVectorHandle
#define IRQ_96_HANDLE                   defaultVectorHandle
#define IRQ_97_HANDLE                   defaultVectorHandle
#define IRQ_98_HANDLE                   defaultVectorHandle
#define IRQ_99_HANDLE                   defaultVectorHandle

#define IRQ_100_HANDLE                  defaultVectorHandle
#define IRQ_101_HANDLE                  defaultVectorHandle
#define IRQ_102_HANDLE                  defaultVectorHandle
#define IRQ_103_HANDLE                  defaultVectorHandle
#define IRQ_104_HANDLE                  defaultVectorHandle
#define IRQ_105_HANDLE                  defaultVectorHandle
#define IRQ_106_HANDLE                  defaultVectorHandle
#define IRQ_107_HANDLE                  defaultVectorHandle
#define IRQ_108_HANDLE                  defaultVectorHandle
#define IRQ_109_HANDLE                  defaultVectorHandle

#define IRQ_110_HANDLE                  defaultVectorHandle
#define IRQ_111_HANDLE                  defaultVectorHandle
#define IRQ_112_HANDLE                  defaultVectorHandle
#define IRQ_113_HANDLE                  defaultVectorHandle
#define IRQ_114_HANDLE                  defaultVectorHandle
#define IRQ_115_HANDLE                  defaultVectorHandle
#define IRQ_116_HANDLE                  defaultVectorHandle
#define IRQ_117_HANDLE                  defaultVectorHandle
#define IRQ_118_HANDLE                  defaultVectorHandle
#define IRQ_119_HANDLE                  defaultVectorHandle

#define IRQ_120_HANDLE                  defaultVectorHandle
#define IRQ_121_HANDLE                  defaultVectorHandle
#define IRQ_122_HANDLE                  defaultVectorHandle
#define IRQ_123_HANDLE                  defaultVectorHandle
#define IRQ_124_HANDLE                  defaultVectorHandle
#define IRQ_125_HANDLE                  defaultVectorHandle
#define IRQ_126_HANDLE                  defaultVectorHandle
#define IRQ_127_HANDLE                  defaultVectorHandle
#define IRQ_128_HANDLE                  defaultVectorHandle
#define IRQ_129_HANDLE                  defaultVectorHandle

#define IRQ_130_HANDLE                  defaultVectorHandle
#define IRQ_131_HANDLE                  defaultVectorHandle
#define IRQ_132_HANDLE                  defaultVectorHandle
#define IRQ_133_HANDLE                  defaultVectorHandle
#define IRQ_134_HANDLE                  defaultVectorHandle
#define IRQ_135_HANDLE                  defaultVectorHandle
#define IRQ_136_HANDLE                  defaultVectorHandle
#define IRQ_137_HANDLE                  defaultVectorHandle
#define IRQ_138_HANDLE                  defaultVectorHandle
#define IRQ_139_HANDLE                  defaultVectorHandle

#define IRQ_140_HANDLE                  defaultVectorHandle
#define IRQ_141_HANDLE                  defaultVectorHandle
#define IRQ_142_HANDLE                  defaultVectorHandle
#define IRQ_143_HANDLE                  defaultVectorHandle
#define IRQ_144_HANDLE                  defaultVectorHandle
#define IRQ_145_HANDLE                  defaultVectorHandle
#define IRQ_146_HANDLE                  defaultVectorHandle
#define IRQ_147_HANDLE                  defaultVectorHandle
#define IRQ_148_HANDLE                  defaultVectorHandle
#define IRQ_149_HANDLE                  defaultVectorHandle

#define IRQ_150_HANDLE                  defaultVectorHandle
#define IRQ_151_HANDLE                  defaultVectorHandle
#define IRQ_152_HANDLE                  defaultVectorHandle
#define IRQ_153_HANDLE                  defaultVectorHandle
#define IRQ_154_HANDLE                  defaultVectorHandle
#define IRQ_155_HANDLE                  defaultVectorHandle
#define IRQ_156_HANDLE                  defaultVectorHandle
#define IRQ_157_HANDLE                  defaultVectorHandle
#define IRQ_158_HANDLE                  defaultVectorHandle
#define IRQ_159_HANDLE                  defaultVectorHandle

#define IRQ_160_HANDLE                  defaultVectorHandle
#define IRQ_161_HANDLE                  defaultVectorHandle
#define IRQ_162_HANDLE                  defaultVectorHandle
#define IRQ_163_HANDLE                  defaultVectorHandle
#define IRQ_164_HANDLE                  defaultVectorHandle
#define IRQ_165_HANDLE                  defaultVectorHandle
#define IRQ_166_HANDLE                  defaultVectorHandle
#define IRQ_167_HANDLE                  defaultVectorHandle
#define IRQ_168_HANDLE                  defaultVectorHandle
#define IRQ_169_HANDLE                  defaultVectorHandle

#define IRQ_170_HANDLE                  defaultVectorHandle
#define IRQ_171_HANDLE                  defaultVectorHandle
#define IRQ_172_HANDLE                  defaultVectorHandle
#define IRQ_173_HANDLE                  defaultVectorHandle
#define IRQ_174_HANDLE                  defaultVectorHandle
#define IRQ_175_HANDLE                  defaultVectorHandle
#define IRQ_176_HANDLE                  defaultVectorHandle
#define IRQ_177_HANDLE                  defaultVectorHandle
#define IRQ_178_HANDLE                  defaultVectorHandle
#define IRQ_179_HANDLE                  defaultVectorHandle

#define IRQ_180_HANDLE                  defaultVectorHandle
#define IRQ_181_HANDLE                  defaultVectorHandle
#define IRQ_182_HANDLE                  defaultVectorHandle
#define IRQ_183_HANDLE                  defaultVectorHandle
#define IRQ_184_HANDLE                  defaultVectorHandle
#define IRQ_185_HANDLE                  defaultVectorHandle
#define IRQ_186_HANDLE                  defaultVectorHandle
#define IRQ_187_HANDLE                  defaultVectorHandle
#define IRQ_188_HANDLE                  defaultVectorHandle
#define IRQ_189_HANDLE                  defaultVectorHandle
#define IRQ_190_HANDLE                  defaultVectorHandle

#endif                                                                  /*  __VIC_CFG_H                 */

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
