#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "CAR.h"
#include "Servo.h"
#include "Ultrasound.h"
#include "Track.h"
#include "Key.h"
#include "Tracking.h"
#include "Anticollision.h"
#include "PWMServo.h"

uint16_t Data1;
uint8_t KeyNum, Num1, Num2;

int main(void)
{
	/*模块初始化*/
	Car_Init();
	Servo_Init();
	Ultrasound_Init();
	Infrared_Init();
	Key_Init();
	OLED_Init();
	PWM_Init2();
	
	OLED_ShowString(1, 3, "Welcome back.");
	
	while (1)
	{
		KeyNum = Key_GetNum();
		if (KeyNum == 1){
			Num1 += 1; 
			if (Num1 > 2){ 
				Num1 = 1;
			}
		}
		if (KeyNum == 2){
			Num2 += 1; 
			if (Num2 > 2){ 
				Num2 = 1;
			}
		} 
		OLED_ShowNum(4, 7, Num1, 1);
		OLED_ShowNum(4, 10, Num2, 1); 
		/*调试到此*/
		
		if (Num2 == 2){ 
			OLED_ShowString(3, 1, "       OFF      "); 
		} 
		else if (Num2 == 1){
			OLED_ShowString(3, 1, "       ON       ");
		}
		else if (Num1 == 0){
			OLED_ShowString(2, 1, "    Standby     ");
			OLED_ShowString(3, 1, "    Standby     ");
		}
		
		/*避障模块*/ 
		if (Num1 == 1)
		{
			OLED_ShowString(2, 2, "Anticollision"); 
			if (Num2 == 1) 
			{ 
				
				/*程序代码_起始*/ 

				Anticollision();
				
				/*程序代码_结束*/
			}
			else if (Num2 == 2){
				Car_Stop();
			}
		}
		
		/*循迹模块*/ 
		if (Num1 == 2)
		{
			OLED_ShowString(2, 1, "    Tracking    "); 
			if (Num2 == 1) 
			{ 
				
				/*程序代码_起始*/ 

				Tracking();
				
				/*程序代码_结束*/
			}
			else if (Num2 == 2){
				Car_Stop();
			}
		}
	}
}
