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

#include "pid.h"

//float Velocity_KP = 0.18f,Velocity_KI = 0.17f;	//����ʽPI���������ڵ���ٶȿ��ƣ�������������
float Velocity_KP = 200,Velocity_KI = 140;			//����ʽPI���������ڵ���ٶȿ��ƣ���������������

//�״�С����ֱ�߸�����PID����
float Diff_Along_Distance_KP = 0.00080f,Diff_Along_Distance_KD = 120.0f,Diff_Along_Distance_KI = 0.00001f;
float Akm_Along_Distance_KP = 0.000415f,Akm_Along_Distance_KD = 1000.245f,Akm_Along_Distance_KI = 0.00001f;	
float STank_Along_Distance_KP = 0.018,STank_Along_Distance_KD = 6.8880f,STank_Along_Distance_KI = 0.00001f;
float BTank_Along_Distance_KP = 0.018,BTank_Along_Distance_KD = 2.8880f,BTank_Along_Distance_KI = 0.00001f;


//�״���������PID����
float Follow_KP_BTank = 0.01072f,Follow_KD_BTank = 0.0461f,Follow_KI_BTank = 0.0001f;	
float Follow_KP_Akm = 0.015f,Follow_KD_Akm = 0.0182f,Follow_KI_Akm = -0.0001f;			
float Follow_KP_STank = 0.0138f,Follow_KD_STank = 0.0446f,Follow_KI_STank = 0.0001f;	
float Follow_KP_Diff = 0.01f,Follow_KD_Diff = 0.025f,Follow_KI_Diff = 0.0001f;		

float Distance_KP =0.001685,Distance_KD = 0.25557 ,Distance_KI = 0.00001;	//�������PID����


//�����͵��Ѳ��PID����,����K�Ƿǳ���PID��һ����������ڷ����Ա仯
float ELE_KP_Diff = 0.01642f,ELE_KD_Diff = 3.0f,ELE_KI_Diff = 0.00012f,ELE_K_Diff = 0.00018f;
float ELE_KP_Akm = 0.00508f,ELE_KD_Akm = 0.008f,ELE_KI_Akm = 0.00012f,ELE_K_Akm = 0.00028f;
float ELE_KP_STank = 0.04368,ELE_KD_STank = 3.41,ELE_KI_STank = 0.0002,ELE_K_STank = 0.0002;
float ELE_KP_BTank = 0.01058,ELE_KD_BTank = 4.182,ELE_KI_BTank = 0.0005,ELE_K_BTank = 0.0004; 


//������CCDѲ��PID����
float CCD_KP_Akm = 0.0126,CCD_KD_Akm = 0.0262,CCD_KI_Akm = 0.0001;
float CCD_KP_Diff = 0.02166,CCD_KD_Diff = 0.3100,CCD_KI_Diff = 0.0001;
float CCD_KP_STank = 0.05772,CCD_KD_STank = 0.01186,CCD_KI_STank = 0.0001;
float CCD_KP_BTank = 0.03345,CCD_KD_BTank = 0.01902,CCD_KI_BTank = 0.0001;


/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)��������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/

int Incremental_PI_Left (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//����ʽPI������
//	 if(Pwm>7200)Pwm=7200;
//	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   					//������һ��ƫ�� 
	 return Pwm;                         					//�������
}


int Incremental_PI_Right (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//����ʽPI������
//	 if(Pwm>7200)Pwm=7200;
//	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   					//������һ��ƫ�� 
	 return Pwm;                         					//�������
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
�������ܣ���ֱ���״����pid
��ڲ���: ��ǰ�����Ŀ�����
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	
//��ֱ���״�������pid

float Along_Adjust_PID(float Current_Distance,float Target_Distance)//�������PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//����ƫ��
	Integral_bias+=Bias;	                                 			//���ƫ��Ļ���
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Num == Diff_Car)
		OutPut=-Diff_Along_Distance_KP*Bias-Diff_Along_Distance_KI*Integral_bias-Diff_Along_Distance_KD*(Bias-Last_Bias);//λ��ʽPID������
	else if(Car_Num == Akm_Car)
		OutPut=-Akm_Along_Distance_KP*Bias-Akm_Along_Distance_KI*Integral_bias-Akm_Along_Distance_KD*(Bias-Last_Bias);//λ��ʽPID������
	else if(Car_Num == Small_Tank_Car)
		OutPut=-STank_Along_Distance_KP*Bias-STank_Along_Distance_KI*Integral_bias-STank_Along_Distance_KD*(Bias-Last_Bias);
	else
		OutPut=-BTank_Along_Distance_KP*Bias-BTank_Along_Distance_KI*Integral_bias-BTank_Along_Distance_KD*(Bias -Last_Bias);
	Last_Bias=Bias;                                       		 			//������һ��ƫ��
	if(MotorA.Motor_Pwm == 0 && MotorB.Motor_Pwm == 0)								//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                           					 	//���                                        	
}


/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
�������ܣ��״�ת��pid
��ڲ���: ��ǰ�ǶȺ�Ŀ��Ƕ�
����  ֵ�����ת���ٶ�
**************************************************************************/	 	
//�״�ת��pid
float Follow_Turn_PID(float Current_Angle,float Target_Angle)
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Angle-Current_Angle;                         				 //����ƫ��
	Integral_bias+=Bias;	                                 				 //���ƫ��Ļ���
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Num == Akm_Car)//��������ר�ò���
		OutPut=Follow_KP_Akm*Bias+Follow_KI_Akm*Integral_bias+Follow_KD_Akm*(Bias-Last_Bias);//λ��ʽPID������
	else if(Car_Num == Small_Tank_Car)
		OutPut=Follow_KP_STank*Bias+Follow_KI_STank*Integral_bias+Follow_KD_STank*(Bias-Last_Bias);	//λ��ʽPID������
	else if(Car_Num == Diff_Car)
		OutPut=Follow_KP_Diff*Bias+Follow_KI_Diff*Integral_bias+Follow_KD_Diff*(Bias-Last_Bias);	//λ��ʽPID������
	else
		OutPut=Follow_KP_BTank*Bias+Follow_KI_BTank*Integral_bias+Follow_KD_BTank*(Bias-Last_Bias);	//λ��ʽPID������
	Last_Bias=Bias;                                       					 		//������һ��ƫ��
	if(MotorA.Motor_Pwm == 0 && MotorB.Motor_Pwm == 0)								//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                           					 	//���
	
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
�������ܣ��״�ת��pid
��ڲ���: ��ǰ�����Ŀ�����
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	
//�״�������pid
float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//�������PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//����ƫ��
	Integral_bias+=Bias;	                                 			//���ƫ��Ļ���
	if(Integral_bias>1000) Integral_bias=1000.0;
	else if(Integral_bias<-1000) Integral_bias=-1000.0;
	OutPut=-Distance_KP*Bias-Distance_KI*Integral_bias-Distance_KD*(Bias-Last_Bias);//λ��ʽPID������
	Last_Bias=Bias;                                       		 			//������һ��ƫ��
	if(MotorA.Motor_Pwm == 0 && MotorB.Motor_Pwm == 0)						//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                          	
}

/**************************************************************************
Function: ELE_PID
Input   : Current_ELE_ADC;Target_ELE_ADC
Output  : OutPut
�������ܣ����Ѳ��PID
��ڲ���: ��ǰ���Ѳ��ADC��Ŀ��ADC
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	
float ELE_PID(int Current_ELE_ADC,int Target_ELE_ADC )
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_ELE_ADC-Current_ELE_ADC;                        //����ƫ��
	Integral_bias+=Bias;	                                 	//���ƫ��Ļ���
//	if(Integral_bias>5000) Integral_bias=5000;
//	else if(Integral_bias<-5000) Integral_bias=-5000;
	if(Car_Num == Diff_Car)										//���Ͳ�ͬ��������ͬ
		OutPut=-ELE_KP_Diff*Bias-ELE_KI_Diff*Integral_bias-ELE_KD_Diff*(Bias-Last_Bias)-ELE_K_Diff*myabs(Bias)*Bias;//λ��ʽPID������
	else if(Car_Num == Akm_Car)
		OutPut=-ELE_KP_Akm*Bias-ELE_KD_Akm*(Bias-Last_Bias);//-ELE_K_Akm*myabs(Bias)*Bias;
	else if(Car_Num == Small_Tank_Car)
		OutPut=-ELE_KP_STank*Bias-ELE_KI_STank*Integral_bias-ELE_KD_STank*(Bias-Last_Bias)-ELE_K_STank*myabs(Bias)*Bias;
	else																					
		OutPut=-ELE_KP_BTank*Bias-ELE_KI_BTank*Integral_bias-ELE_KD_BTank*(Bias-Last_Bias)-ELE_K_BTank*myabs(Bias)*Bias;
	Last_Bias=Bias;                                       		//������һ��ƫ��
	if(MotorA.Motor_Pwm == 0 && MotorB.Motor_Pwm == 0)			//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                          	//���
}
/**************************************************************************
Function: CCD_PID
Input   : Current_Value;Target_Value
Output  : OutPut
�������ܣ�CCDѲ��PID
��ڲ���: ��ǰCCD��ֵ��Ŀ��ֵ
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	
float CCD_PID(float Current_Value,float Target_Value )
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Value-Current_Value;                         	 	//����ƫ��
	Integral_bias+=Bias;	                                 		//���ƫ��Ļ���
	if(Integral_bias>5000) Integral_bias=5000;
	else if(Integral_bias<-5000) Integral_bias=-5000;
	if(Car_Num == Akm_Car)
		OutPut=(CCD_KP_Akm)*Bias+(CCD_KI_Akm)*Integral_bias+(CCD_KD_Akm)*(Bias-Last_Bias);//λ��ʽPID������
	else if(Car_Num == Diff_Car)
		OutPut=CCD_KP_Diff*Bias+CCD_KI_Diff*Integral_bias+CCD_KD_Diff*(Bias-Last_Bias);
	else if(Car_Num == Small_Tank_Car)
		OutPut=CCD_KP_STank*Bias+CCD_KI_STank*Integral_bias+CCD_KD_STank*(Bias-Last_Bias);
	else
		OutPut=CCD_KP_BTank*Bias+CCD_KI_BTank*Integral_bias+CCD_KD_BTank*(Bias-Last_Bias);
	Last_Bias=Bias;                                       		//������һ��ƫ��
	if(MotorA.Motor_Pwm == 0 && MotorB.Motor_Pwm == 0)			//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                       
}



