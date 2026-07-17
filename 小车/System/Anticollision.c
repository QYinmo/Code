#include "stm32f10x.h"                  // Device header
#include "Ultrasound.h"
#include "Delay.h"
#include "CAR.h"
#include "Servo.h"

	
	
void Anticollision(void)
	{
		Go_Ahead();
		
		uint16_t a = Test_Distance();
		
		if(a<35)
		{
			Car_Stop();
			Servo_SetAngle(0);
			Delay_ms(1000);
			
			uint16_t b= Test_Distance();		
			if(b>35)
			{
				Servo_SetAngle(90);
				Delay_ms(1000);
				Self_Right();
				Delay_ms(300);
				Go_Ahead();
			
			}
			else 
			{
				Servo_SetAngle(180);
				Delay_ms(1000);
				uint16_t c= Test_Distance();
				
				if(c>35)
				{	
					Servo_SetAngle(90);
					Delay_ms(1000);
					Self_Left();
					Delay_ms(300);
					Go_Ahead();
				}
				else
				{
					Servo_SetAngle(90);
					Delay_ms(1000);
					Go_Back();
					Delay_ms(500);
					Car_Stop();
				}
			}
		}
		Delay_ms(100);
	}
