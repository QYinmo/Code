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

#ifndef __USART_H
#define	__USART_H


#include "stm32f10x.h"
#include <stdio.h>
#include "Header.h"

/** 
  * ���ں궨�壬��ͬ�Ĵ��ڹ��ص����ߺ�IO��һ������ֲʱ��Ҫ�޸��⼸����
	* 1-�޸�����ʱ�ӵĺ꣬uart1���ص�apb2���ߣ�����uart���ص�apb1����
	* 2-�޸�GPIO�ĺ�
  */
	
//����1-USART1
//���Դ���

#define  DEBUG_USARTx                   USART1
#define  DEBUG_USART_CLK                RCC_APB2Periph_USART1
#define  DEBUG_USART_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  DEBUG_USART_BAUDRATE           115200

// USART GPIO ���ź궨��
#define  DEBUG_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  DEBUG_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  DEBUG_USART_TX_GPIO_PORT       GPIOA   
#define  DEBUG_USART_TX_GPIO_PIN        GPIO_Pin_9
#define  DEBUG_USART_RX_GPIO_PORT       GPIOA
#define  DEBUG_USART_RX_GPIO_PIN        GPIO_Pin_10

#define  DEBUG_USART_IRQ                USART1_IRQn
#define  DEBUG_USART_IRQHandler         USART1_IRQHandler



 //����2-USART2
 //���д���
#define  WIRELESS_USARTx                   USART2
#define  WIRELESS_USART_CLK                RCC_APB1Periph_USART2
#define  WIRELESS_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  WIRELESS_USART_BAUDRATE           115200

// USART GPIO ���ź궨��
#define  WIRELESS_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  WIRELESS_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  WIRELESS_USART_TX_GPIO_PORT       GPIOA   
#define  WIRELESS_USART_TX_GPIO_PIN        GPIO_Pin_2
#define  WIRELESS_USART_RX_GPIO_PORT       GPIOA
#define  WIRELESS_USART_RX_GPIO_PIN        GPIO_Pin_3

#define  WIRELESS_USART_IRQ                USART2_IRQn
#define  WIRELESS_USART_IRQHandler         USART2_IRQHandler




// ����3-USART3
//�����Ĵ���
#define  BLUETOOTH_USARTx                   USART3
#define  BLUETOOTH_USART_CLK                RCC_APB1Periph_USART3
#define  BLUETOOTH_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  BLUETOOTH_USART_BAUDRATE           115200

// USART GPIO ���ź궨��
#define  BLUETOOTH_USART_GPIO_CLK           (RCC_APB2Periph_GPIOB)
#define  BLUETOOTH_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  BLUETOOTH_USART_TX_GPIO_PORT       GPIOB   
#define  BLUETOOTH_USART_TX_GPIO_PIN        GPIO_Pin_10
#define  BLUETOOTH_USART_RX_GPIO_PORT       GPIOB
#define  BLUETOOTH_USART_RX_GPIO_PIN        GPIO_Pin_11

#define  BLUETOOTH_USART_IRQ                USART3_IRQn
#define  BLUETOOTH_USART_IRQHandler         USART3_IRQHandler



//// ����4-UART4
////Ԥ��openmv
//#define  OPENMV_USARTx                   UART4
//#define  OPENMV_USART_CLK                RCC_APB1Periph_UART4
//#define  OPENMV_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
//#define  OPENMV_USART_BAUDRATE           115200

//// USART GPIO ���ź궨��
//#define  OPENMV_USART_GPIO_CLK           (RCC_APB2Periph_GPIOC)
//#define  OPENMV_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
//    
//#define  OPENMV_USART_TX_GPIO_PORT       GPIOC   
//#define  OPENMV_USART_TX_GPIO_PIN        GPIO_Pin_10
//#define  OPENMV_USART_RX_GPIO_PORT       GPIOC
//#define  OPENMV_USART_RX_GPIO_PIN        GPIO_Pin_11

//#define  OPENMV_USART_IRQ                UART4_IRQn
//#define  OPENMV_USART_IRQHandler         UART4_IRQHandler




//����5-UART5
//�״ﴮ��
#define  LIDAR_USARTx                   UART5
#define  LIDAR_USART_CLK                RCC_APB1Periph_UART5
#define  LIDAR_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  LIDAR_USART_BAUDRATE           115200

// USART GPIO ���ź궨��
#define  LIDAR_USART_GPIO_CLK           (RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD)
#define  LIDAR_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  LIDAR_USART_TX_GPIO_PORT       GPIOC   
#define  LIDAR_USART_TX_GPIO_PIN        GPIO_Pin_12
#define  LIDAR_USART_RX_GPIO_PORT       GPIOD
#define  LIDAR_USART_RX_GPIO_PIN        GPIO_Pin_2

#define  LIDAR_USART_IRQ                UART5_IRQn
#define  LIDAR_USART_IRQHandler         UART5_IRQHandler




void DEBUG_USART_Init(void);
void BLUETOOTH_USART_Init(void);
void LIDAR_USART_Init(void);
void WIRELESS_USART_Init(void);

#endif /* __USART_H */
