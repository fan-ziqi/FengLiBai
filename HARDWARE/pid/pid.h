//#pragma once

#ifndef __PID_H
#define __PID_H	 

struct _pid
{
	float Setangle;  //定义设定值
	float Now_err;   //定义当前误差
  float Err_k1;    //上一次误差
	float Err_k2;    //上上次误差
	float kp,ki,kd;  //定义比列，微分，积分常数
	
}pid;

void pid_init(void);
float pid_realize(float angle);

void pid_init(void)
{
	pid.Setangle=30.0;
	pid.Now_err=0.0;
	pid.Err_k1=0.0;
	pid.Err_k2=0.0;
	pid.kp=6.00;
	pid.ki=0.40;
	pid.kd=0.60;
}

float pid_realize(float angle)
{
	float ans;
	pid.Now_err=pid.Setangle-angle;
	ans=pid.kp*(pid.Now_err-pid.Err_k1)+pid.ki*pid.Now_err+pid.kd*(pid.Now_err-2*pid.Err_k1+pid.Err_k2);
	pid.Err_k2=pid.Err_k1;
	pid.Err_k1=pid.Now_err;
	return ans;
}

#endif

