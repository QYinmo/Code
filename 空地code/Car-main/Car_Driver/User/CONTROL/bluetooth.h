/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2023-03-02

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2023-03-02

All rights reserved
***********************************************/


#ifndef __BLUETOOTH_H
#define	__BLUETOOTH_H

#include "stm32f10x.h"
#include "Header.h"
#include "math.h"
#include <string.h>



typedef struct
{	
	u8 start_flag;
	
}Wireless_Send_data;

typedef struct
{
	short X;
	short Y;
}Fire_Pos;

typedef struct
{
	short X;
	short Y;
}Current_Pos;

typedef struct
{
	u8 option;
	Fire_Pos fire_pos;
	Current_Pos current_pos;
}Wireless_Receive_Data;

typedef struct
{
	float Vel_X;
	float Vel_Z;
	u8 start_flag;
}Control_Frame;

typedef struct
{
		short X_speed;	            //2 bytes //2个字节
		short Y_speed;              //2 bytes //2个字节
		short Z_speed;
		Wireless_Receive_Data wire_less_data;
		u8 option;
		u8 buffer[10];
} Bluetooth_Send_data;

extern u8 PID_Send;//PID
extern u8 Flag_Direction;
extern Control_Frame control_frame;
void data_anal(u8*data);
void data_anal_wireless(u8* data);

#endif
