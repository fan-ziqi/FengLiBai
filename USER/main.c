//////////////////////////////////////////////////////////////////////////////////	 
//ALIENTEK��ӢSTM32������V3
//�������ϵͳ	   
//���ߣ�������
//��������:2020/9/5
//�汾��V1.0								  
////////////////////////////////////////////////////////////////////////////////// 
#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"
#include "mpu6050.h"
#include "usmart.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "timer.h"
#include "pwm.h"
#include "pid.h"




////����1����1���ַ� 
////c:Ҫ���͵��ַ�
//void usart1_send_char(u8 c)
//{   	
//	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
//	USART_SendData(USART1,c);  
//} 
////�������ݸ�����������λ�����(V2.6�汾)
////fun:������. 0XA0~0XAF
////data:���ݻ�����,���28�ֽ�!!
////len:data����Ч���ݸ���
//void usart1_niming_report(u8 fun,u8*data,u8 len)
//{
//	u8 send_buf[32];
//	u8 i;
//	if(len>28)return;	//���28�ֽ����� 
//	send_buf[len+3]=0;	//У��������
//	send_buf[0]=0X88;	//֡ͷ
//	send_buf[1]=fun;	//������
//	send_buf[2]=len;	//���ݳ���
//	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
//	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���	
//	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
//}
////���ͼ��ٶȴ��������ݺ�����������
////aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
////gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
//{
//	u8 tbuf[12]; 
//	tbuf[0]=(aacx>>8)&0XFF;
//	tbuf[1]=aacx&0XFF;
//	tbuf[2]=(aacy>>8)&0XFF;
//	tbuf[3]=aacy&0XFF;
//	tbuf[4]=(aacz>>8)&0XFF;
//	tbuf[5]=aacz&0XFF; 
//	tbuf[6]=(gyrox>>8)&0XFF;
//	tbuf[7]=gyrox&0XFF;
//	tbuf[8]=(gyroy>>8)&0XFF;
//	tbuf[9]=gyroy&0XFF;
//	tbuf[10]=(gyroz>>8)&0XFF;
//	tbuf[11]=gyroz&0XFF;
//	usart1_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
//}	
////ͨ������1�ϱ���������̬���ݸ�����
////aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
////gyrox,gyroy,gyroz:x,y,z�������������������ֵ
////roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
////pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
////yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
//void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
//{
//	u8 tbuf[28]; 
//	u8 i;
//	for(i=0;i<28;i++)tbuf[i]=0;//��0
//	tbuf[0]=(aacx>>8)&0XFF;
//	tbuf[1]=aacx&0XFF;
//	tbuf[2]=(aacy>>8)&0XFF;
//	tbuf[3]=aacy&0XFF;
//	tbuf[4]=(aacz>>8)&0XFF;
//	tbuf[5]=aacz&0XFF; 
//	tbuf[6]=(gyrox>>8)&0XFF;
//	tbuf[7]=gyrox&0XFF;
//	tbuf[8]=(gyroy>>8)&0XFF;
//	tbuf[9]=gyroy&0XFF;
//	tbuf[10]=(gyroz>>8)&0XFF;
//	tbuf[11]=gyroz&0XFF;	
//	tbuf[18]=(roll>>8)&0XFF;
//	tbuf[19]=roll&0XFF;
//	tbuf[20]=(pitch>>8)&0XFF;
//	tbuf[21]=pitch&0XFF;
//	tbuf[22]=(yaw>>8)&0XFF;
//	tbuf[23]=yaw&0XFF;
//	usart1_niming_report(0XAF,tbuf,28);//�ɿ���ʾ֡,0XAF
//}  
 	
 int main(void)
 {	 
	u8 t=0;
//	u8 report=1;			//Ĭ�Ͽ����ϱ�
	u16 pwmval = 0;
	u8 dir = 1;
	int mode=1;
	u16 speed=0;
	u8 now_speed;
	u8 key=0;
	float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
//	short temp;					//�¶�	
	short pitch10;					//�¶�	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(500000);	 	//���ڳ�ʼ��Ϊ500000
	delay_init();	//��ʱ��ʼ�� 
	usmart_dev.init(72);		//��ʼ��USMART
	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();					//��ʼ������
	LCD_Init();			   		//��ʼ��LCD  
	pid_init();
	MPU_Init();					//��ʼ��MPU6050
 	POINT_COLOR=RED;			//��������Ϊ��ɫ 
 	POWER_M();
	TIM3_MOTOR_PWM_Init(99,720); //����Ƶ��PWMƵ��=72000/(899+1)=80Khz
	TIM2_MOTOR_PWM_Init(99,720); //����Ƶ��PWMƵ��=72000/(899+1)=80Khz
	GPIO_SetBits(MOTOR_IN1_PORT,   MOTOR1_IN1);
	GPIO_ResetBits(MOTOR_IN1_PORT, MOTOR1_IN2);
	GPIO_SetBits(MOTOR_IN1_PORT,   MOTOR2_IN1);
	GPIO_ResetBits(MOTOR_IN1_PORT, MOTOR2_IN2);
	GPIO_SetBits(MOTOR_IN1_PORT,   MOTOR3_IN1);
	GPIO_ResetBits(MOTOR_IN1_PORT, MOTOR3_IN2);
	GPIO_SetBits(MOTOR_IN1_PORT,   MOTOR4_IN1);
	GPIO_ResetBits(MOTOR_IN1_PORT, MOTOR4_IN2);

	while(mpu_dmp_init())
 	{
		LCD_ShowString(30,130,200,16,16,"MPU6050 Error");
		delay_ms(200);
		LCD_Fill(30,130,239,130+16,WHITE);
 		delay_ms(200);
	}  
	LCD_ShowString(30,130,200,16,16,"MPU6050 OK");
	POINT_COLOR=BLUE;//��������Ϊ��ɫ  
 	LCD_ShowString(30,170,200,16,16,"P");	 
 	LCD_ShowString(30,200,200,16,16," Temp:    . C");	
 	LCD_ShowString(30,220,200,16,16,"Pitch:    . C");	
 	LCD_ShowString(30,240,200,16,16," Roll:    . C");	 
 	LCD_ShowString(30,260,200,16,16," Yaw :    . C");	 
 	LCD_ShowString(30,280,200,16,16," kp:0.  ");	 
	LCD_ShowString(30,300,200,16,16," ki :0.      ");	 
	LCD_ShowString(30,320,200,16,16," kd :0.     ");
  LCD_ShowString(30,340,200,16,16," speed:       ");
	LCD_ShowString(30,360,200,16,16," setangle:     ");

// 	while(1)
//	{
//		key=KEY_Scan(1);
//		if(key==3)
//		{
//			if(mode==3) mode=1;
//			else mode+=1;
//		}
//		if(mode==1)
//		{
//			LCD_ShowString(30,170,200,16,16,"P");	
//			if(key==1) pid.kp-=0.03;
//			else if(key==2) pid.kp+=0.03;
//		}
//		if(mode==2)
//		{
//			LCD_ShowString(30,170,200,16,16,"I");	 
//			if(key==1) pid.ki-=0.03;
//			else if(key==2) pid.ki+=0.03;
//		}
//		if(mode==3)
//		{
//			LCD_ShowString(30,170,200,16,16,"D");	 
//			if(key==1) pid.kd-=0.03;
//			else if(key==2) pid.kd+=0.03;
//		}
//		
//		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
//		{ 
////			temp=MPU_Get_Temperature();	//�õ��¶�ֵ
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
//			//mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
//			//usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
////			if((t%10)==0)
////			{ 
//				pitch10=pitch*10;
//				if(pitch10<0)
//				{
//					LCD_ShowChar(30+48,220,'-',16,0);		//��ʾ����
//					pitch10=-pitch10;		//תΪ����
//				}
//				else LCD_ShowChar(30+48,220,' ',16,0);		//ȥ������ 
//				LCD_ShowNum(30+48+8,220,pitch10/10,3,16);		//��ʾ��������	    
//				LCD_ShowNum(30+48+40,220,pitch10%10,1,16);		//��ʾС������
//				t=0;
//				LED0=!LED0;//LED��˸
////			}
//		}
//		t++; 
//		now_speed=speed;
//		LCD_ShowNum(30+48+8,280,pid.kp*100,5,16);
//		LCD_ShowNum(30+48+8,300,pid.ki*100,5,16);
//		LCD_ShowNum(30+48+8,320,pid.kd*100,5,16);
//		LCD_ShowNum(30+48+8,340,now_speed,3,16);
//		LCD_ShowNum(30+48+8,360,pid.Setangle,3,16);
//		
//		speed+=pid_realize(pitch);
//		if(speed>=800) speed=800;
//		else if(speed<=100) speed=100;
//		TIM_SetCompare1(TIM1,800-speed);	//�޸ıȽ�ֵ���޸�ռ�ձ�
//	} 	







	while(1)
		{
//			if(dir)pwmval++;//dir==1 led0pwmval����
//			else pwmval--;	//dir==0 led0pwmval�ݼ� 
//			if(pwmval>800)dir=0;//led0pwmval����800�󣬷���Ϊ�ݼ�
//			if(pwmval==0)dir=1;	//led0pwmval�ݼ���0�󣬷����Ϊ����
//	    LCD_ShowNum(30+48+8,340,pwmval,3,16);
			TIM1_PWM_COMPARE(1,50);
			TIM1_PWM_COMPARE(2,50);
			TIM1_PWM_COMPARE(3,50);
			TIM1_PWM_COMPARE(4,50);
			

			
//			TIM_SetCompare1(TIM1,pwmval);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		}
}
 





