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

#include "bluetooth.h"
u8 PID_Send;	   // �����ֻ�app��ȡ��������ı���
u8 Flag_Direction; // �����ֻ�app�ķ��������˳ʱ��һȦ��8��������ֵ��1--8,ֹͣʱ��ֵΪ0
Control_Frame control_frame;
Bluetooth_Send_data bluetooth_send_data;
Wireless_Send_data wireless_send_data;
Wireless_Receive_Data wireless_receive_data;

/*����У���*/
u8 calculate_checksum(u8 *str, int n)
{
	u8 checksum = 0;
	for (int i = 0; i < n; i++)
	{
		checksum += str[i];
	}
	return checksum;
}

/*
���͵���ݮ�ɵ����ݽ����������ݰ��зֳ��ֽ������㷢�͡�
֡ͷ��0xAA,optionΪ0x01����С��״̬��0x02ת��������������
*/
void trans_bluetooth()
{
	bluetooth_send_data.X_speed = ((MotorA.Current_Encoder + MotorB.Current_Encoder) / 2) * 1000;
	bluetooth_send_data.Y_speed = 0;
	bluetooth_send_data.Z_speed = ((MotorB.Current_Encoder - MotorA.Current_Encoder) / Akm_wheelspacing) * 1000;
	bluetooth_send_data.buffer[0] = 0xAA;
	bluetooth_send_data.buffer[1] = bluetooth_send_data.option;
	if (bluetooth_send_data.option == 0x01)
	{
		bluetooth_send_data.buffer[2] = bluetooth_send_data.X_speed >> 8;
		bluetooth_send_data.buffer[3] = bluetooth_send_data.X_speed;
		bluetooth_send_data.buffer[4] = bluetooth_send_data.Y_speed >> 8;
		bluetooth_send_data.buffer[5] = bluetooth_send_data.Y_speed;
		bluetooth_send_data.buffer[6] = bluetooth_send_data.Z_speed >> 8;
		bluetooth_send_data.buffer[7] = bluetooth_send_data.Z_speed;
		bluetooth_send_data.buffer[8] = calculate_checksum(bluetooth_send_data.buffer, 8);
		bluetooth_send_data.buffer[9] = 0xFF;
	}
	else if (bluetooth_send_data.option == 0x02)
	{
		bluetooth_send_data.wire_less_data = wireless_receive_data;
		if (bluetooth_send_data.wire_less_data.option == 0x01)
		{
			bluetooth_send_data.buffer[2] = bluetooth_send_data.wire_less_data.fire_pos.X >> 8;
			bluetooth_send_data.buffer[3] = bluetooth_send_data.wire_less_data.fire_pos.X;
			bluetooth_send_data.buffer[4] = bluetooth_send_data.wire_less_data.fire_pos.Y >> 8;
			bluetooth_send_data.buffer[5] = bluetooth_send_data.wire_less_data.fire_pos.Y;
			bluetooth_send_data.buffer[6] = 0x00;
			bluetooth_send_data.buffer[7] = 0x00;
			bluetooth_send_data.buffer[8] = calculate_checksum(bluetooth_send_data.buffer, 8);
			bluetooth_send_data.buffer[9] = 0xFF;
		}
		else if (bluetooth_send_data.wire_less_data.option == 0x02)
		{
			bluetooth_send_data.buffer[2] = bluetooth_send_data.wire_less_data.current_pos.X >> 8;
			bluetooth_send_data.buffer[3] = bluetooth_send_data.wire_less_data.current_pos.X;
			bluetooth_send_data.buffer[4] = bluetooth_send_data.wire_less_data.current_pos.Y >> 8;
			bluetooth_send_data.buffer[5] = bluetooth_send_data.wire_less_data.current_pos.Y;
			bluetooth_send_data.buffer[6] = 0x00;
			bluetooth_send_data.buffer[7] = 0x00;
			bluetooth_send_data.buffer[8] = calculate_checksum(bluetooth_send_data.buffer, 8);
			bluetooth_send_data.buffer[9] = 0xFF;
		}
	}
}

void bluetooth_usart_send(u8 data)
{
	BLUETOOTH_USARTx->DR = data;
	while ((BLUETOOTH_USARTx->SR & 0x40) == 0)
		;
}

void Bluetooth_Send()
{
	u8 i = 0;
	for (i = 0; i < 8; i++)
	{
		usart1_send(bluetooth_send_data.buffer[i]);
	}
}
/*��ݮ�ɽ������ݽ���,0x01��Ϊ����֡��0x02��Ϊ��������*/
void data_anal(u8 *data)
{

	
		control_frame.Vel_X = data[2];
		control_frame.Vel_Z = data[4]/20.0;
		if (data[3])
		{
			control_frame.Vel_Z = -control_frame.Vel_Z;
		}
	

		control_frame.start_flag = data[1];
	return;
}
/*��ݮ�ɽ��������жϣ�0xAAΪ֡ͷ��0xFFΪ֡β*/
void BLUETOOTH_USART_IRQHandler(void)
{
	static int data_idx = 2;
	static u8 bluetooth_receive;
	static u8 buffer[10];
	static int state = 0;
	if (USART_GetITStatus(BLUETOOTH_USARTx, USART_IT_RXNE) != RESET) // ���յ�����
	{
		bluetooth_receive = USART_ReceiveData(BLUETOOTH_USARTx);
		USART_ClearITPendingBit(BLUETOOTH_USARTx, USART_IT_RXNE);
		switch (state)
		{
		case 0:
			if (bluetooth_receive == 0xAA)
			{
				memset(buffer, 0, sizeof(buffer));
				buffer[0] = bluetooth_receive;
				state = 1;
			}
			break;
		case 1:
			buffer[1] = bluetooth_receive;
			state = 2;
			break;
		case 2:
			buffer[data_idx] = bluetooth_receive;
			data_idx += 1;
			if (data_idx > 5)
			{
				state = 3;
				data_idx = 2;
			}
			break;

		case 3:
			if (bluetooth_receive == 0xFF && buffer[5] == calculate_checksum(buffer, 5))
			{
				data_anal(buffer);
				state = 0;


		}
			break;
		USART_ClearITPendingBit(BLUETOOTH_USARTx, USART_IT_RXNE);
	}
}

/*�����������������ݽ�����0x01����Ϊ��Դλ�ã�0x02����Ϊ��ǰλ��*/
/*void data_anal_wireless(u8 *data)
{
	if (data[1] == 0x01)
	{
		wireless_receive_data.option = 0x01;
		wireless_receive_data.fire_pos.X = (data[2] << 8) | data[3];
		wireless_receive_data.fire_pos.Y = (data[4] << 8) | data[5];
	}
	else if (data[1] == 0x02)
	{
		wireless_receive_data.option = 0x02;
		wireless_receive_data.current_pos.X = (data[2] << 8) | data[3];
		wireless_receive_data.current_pos.Y = (data[4] << 8) | data[5];
	}
}*/

/*���������������ݽ�����0xAAΪ֡ͷ��0xFFΪ֡β*/
/*void WIRELESS_USART_IRQHandler(void)
{
	static int data_idx = 2;
	static u8 wireless_receive;
	static u8 buffer[10];
	static int state = 0;
	if (USART_GetITStatus(WIRELESS_USARTx, USART_IT_RXNE) != RESET) // ���յ�����
	{
		wireless_receive = USART_ReceiveData(WIRELESS_USARTx);
		USART_ClearITPendingBit(WIRELESS_USARTx, USART_IT_RXNE);
		switch (state)
		{
		case 0:
			if (wireless_receive == 0xAA)
			{
				buffer[0] = wireless_receive;
				state = 1;
			}
			break;
		case 1:
			buffer[1] = wireless_receive;
			state = 2;

		case 2:
			buffer[data_idx] = wireless_receive;
			data_idx += 1;
			if (data_idx > 7)
			{
				state = 3;
				data_idx = 2;
			}
			break;

		case 3:
			if (wireless_receive == 0xFF && buffer[7] == calculate_checksum(buffer, 7))
			{
				data_anal_wireless(buffer);
				state = 0;
			}
			break;
		}
		USART_ClearITPendingBit(WIRELESS_USARTx, USART_IT_RXNE);
	}
}

*/
/*void wireless_usart_send(u8 data)
{
	WIRELESS_USARTx->DR = data;
	while ((WIRELESS_USARTx->SR & 0x40) == 0)
		;
}

void wireless_Send()
{
	if (control_frame.start_flag)
	{
		wireless_send_data.start_flag = 0x01;
		wireless_usart_send(wireless_send_data.start_flag);
	}*/
}
