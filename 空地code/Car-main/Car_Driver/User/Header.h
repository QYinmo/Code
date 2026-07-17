/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2023-03-02

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2023-03-02

All rights reserved
***********************************************/

#ifndef __HEADER_H
#define __HEADER_H

//ͷ�ļ�����
#include "stm32f10x.h"
#include "./SysTick/bsp_SysTick.h"
#include "./Led/bsp_led.h"
#include "./beep/bsp_beep.h"   
#include "./key/bsp_key.h"  
#include "./usart/bsp_usart.h"
#include "./GeneralTim/bsp_GeneralTim.h" 
#include "./Motor/bsp_motor.h" 
#include "./OLED/bsp_oled.h"
#include "stdlib.h"
#include "./OLED/oledfont.h"  	 
#include "./CONTROL/show.h"
#include "./encoder/encoder.h"
#include "./adc/bsp_adc.h"
#include "bluetooth.h"  
#include "./PSTWO/pstwo.h"
#include "./DataScope_DP/DataScope_DP.h"
#include "pid.h"
#include "Lidar.h"
#include "control.h"
#include "capture.h"
#include "ELE_CCD.h"
#include "MPU6050.h"
#include "usartx.h"
#include "./OLED/bsp_oled.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ioi2c.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "filter.h"
#include "stdio.h"
extern u16 Voltage;						//��ѹ�������Ŵ�100������
extern u8 Flag_Stop;					//�����ͣ��־λ
extern volatile u8 delay_50,delay_flag; 			//��ʱ����
extern u8 Car_Num;						//���ͺ���ѡ��
extern u8 PS2_ON_Flag,APP_ON_Flag,ROS_ON_Flag,Remote_ON_Flag;		//Ĭ�����з�ʽ������
extern u8 Flag_Show ;					//��ʾ��־λ��Ĭ�Ͽ����������л�����λ��ģʽ����ʱ�ر�
extern float Perimeter; 
extern float Wheelspacing; 
extern u16  DISTANCE,ANGLE;

extern u8 one_lap_data_success_flag,Lidar_Success_Receive_flag,Lidar_flag_count;
extern int lap_count,PointDataProcess_count,test_once_flag,Dividing_point,Avoid_Flag;
//����ѡ��ĺ���
#define    Diff_Car    						  	0
#define    Akm_Car 							 	1
#define    Small_Tank_Car				  		2
#define    Big_Tank_Car							3



//#define Wheel_spacing         0.162f
//#define Diff_wheelSpacing     0.177f

//λ������
// ����ֻ������ GPIO ODR��IDR�������Ĵ�����λ����������ַ�������Ĵ�����û�ж���

//SRAM λ����:    0X2000 0000~0X2010 0000
//SRAM λ��������:0X2200 0000~0X23FF FFFF

//���� λ����:    0X4000 0000~0X4010 0000
//���� λ��������:0X4200 0000~0X43FF FFFF

// �ѡ�λ����ַ+λ��š�ת���ɱ�����ַ�ĺ�
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x02000000+((addr & 0x00FFFFFF)<<5)+(bitnum<<2)) 
/*
 *addr & 0xF0000000��ȡ��ַ�ĸ�4λ��������2����4����������SRAM�������ַ��
 *�����2��+0x02000000��=0X2200 0000������SRAM�������4��+0x02000000��=0X4200 0000����������
 *
 *addr & 0x000FFFFFF�����ε�����λ���൱��-0X2000 0000����-0X4000 0000�������ʾƫ��λ�������ٸ��ֽ�
 *<<5  ����*8*4����Ϊλ����һ����ַ��ʾһ���ֽڣ�һ���ֽ���8��bit��һ��bit�������ͳ�һ���֣���4���ֽ�
 *<<2 ����*4����Ϊһ��λ�������ͳ�һ���֣���4���ֽ�
 *
 *�ֽ��������ʽӦ�þ���������
 *SRAMλ��������ַ
 *AliasAddr= 0x22000000+((A-0x20000000)*8+n)*4 =0x22000000+ (A-0x20000000)*8*4 +n*4
 *����λ��������ַ
 *AliasAddr= 0x22000000+((A-0x20000000)*8+n)*4 =0x22000000+ (A-0x20000000)*8*4 +n*4
 */

/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)		 {p->BSRR=i;}	 	//���Ϊ�ߵ�ƽ		
#define digitalLo(p,i)		 {p->BRR=i;}	 	//����͵�ƽ
#define digitalToggle(p,i) {p->ODR ^=i;} 		//�����ת״̬


// ��һ����ַת����һ��ָ��
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 


// ��λ����������ַת����ָ��
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))   


// GPIO ODR �� IDR �Ĵ�����ַӳ�� 
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C   
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C   
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C   
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C   
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C   
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C      
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C      
  
#define GPIOA_IDR_Addr    (GPIOA_BASE+8)  //0x40010808   
#define GPIOB_IDR_Addr    (GPIOB_BASE+8)  //0x40010C08   
#define GPIOC_IDR_Addr    (GPIOC_BASE+8)  //0x40011008   
#define GPIOD_IDR_Addr    (GPIOD_BASE+8)  //0x40011408   
#define GPIOE_IDR_Addr    (GPIOE_BASE+8)  //0x40011808   
#define GPIOF_IDR_Addr    (GPIOF_BASE+8)  //0x40011A08   
#define GPIOG_IDR_Addr    (GPIOG_BASE+8)  //0x40011E08 



// �������� GPIO��ĳһ��IO�ڣ�n(0,1,2...16),n��ʾ��������һ��IO��
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //���   
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //����   
  
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //���   
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //����   
  
#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //���   
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //����   
  
#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //���   
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //����   
  
#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //���   
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����  
  
#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //���   
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����  
  
#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //���   
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����  



#endif 
