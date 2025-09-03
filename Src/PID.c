#include "stdint.h"
float M6020_MOTOR_POSITION_PID_KP=600.0;
float M6020_MOTOR_POSITION_PID_KI=0.1;
float M6020_MOTOR_POSITION_PID_KD=10.0;

float M6020_MOTOR_SPEED_PID_KP=20.0;
float M6020_MOTOR_SPEED_PID_KI=0.01;
float M6020_MOTOR_SPEED_PID_KD=0.0;

#define M6020_MOTOR_POSITION_PID_MAX_OUT 25000.0//这两个只能用来控速度的，主要用于位置环
#define M6020_MOTOR_POSITION_PID_MAX_IOUT 2000.0


typedef int16_t motor_measure;

typedef struct
{
    /*PID三参数*/
    float KP;
    float KI;
    float KD;

    float max_out;//最大输出
    float max_Iout;//最大积分输出

    float Set;//设定值
    float Feedback;//反馈值

    float out;
    float Pout;
    float Iout;
    float Dout;

    float error[3];//用来存储这次的误差，上次的误差和上上次的误差

}PID_type_def;//用数组的话，不直观就这样改了

/*实例化电机*/
typedef struct MOTOR
{
    const motor_measure motor_measure_data[4];
    PID_type_def speed_PID;
    PID_type_def position_PID;
    float position;
    float Set_position;
    float speed;
    float Set_speed;
    int give_Voltage;
}MOTOR_t;

float LimitMax(float out,float max_out)
{
    if(out>max_out)
    {
        out=max_out;
    }
    else if (out<(-max_out))
    {
        out=(-max_out);
    }
    return out;
}

/*功能：PID位置环输出值的计算
参数:pid（哪个环）
now_value:现在的值
set_value:设定值
返回：PID输出的结果
*/
float PID_Position_Calculate(PID_type_def *pid,float now_value,float set_value)
{
    if(pid==0)//留下断电保护的代码
    {
        return 0.0f;
    }

    pid->error[2]=pid->error[1];
    pid->error[1]=pid->error[0];
    pid->error[0]=set_value-now_value;
    pid->Set=set_value;
    pid->Feedback=now_value;

    //Kp：
    pid->Pout=pid->KP*pid->error[0];
    //KI:
    pid->Iout+=pid->KI*pid->error[0];
    pid->Iout=LimitMax(pid->Iout,M6020_MOTOR_POSITION_PID_MAX_IOUT);
    //KD:
    pid->Dout=pid->KD*(pid->error[0]-pid->error[1]);

    pid->out=pid->Pout+pid->Iout+pid->Dout;
    pid->out=LimitMax(pid->out,M6020_MOTOR_POSITION_PID_MAX_OUT);

    return pid->out;
}

/*功能：PID速度环输出值的计算
参数:pid（哪个环）
now_value:现在的值
set_value:设定值
返回：PID输出的结果
*/
float PID_Speed_Calculate(PID_type_def *pid,float now_value,float set_value)
{
    if(pid==0)//留下断电保护的代码
    {
        return 0.0f;
    }

    pid->error[0]=set_value-now_value;
    pid->Set=set_value;
    pid->Feedback=now_value;

    //Kp：
    pid->Pout=pid->KP*(pid->error[0]-pid->error[1]);
    //KI:
    pid->Iout=pid->KI*pid->error[0];
    pid->Iout=LimitMax(pid->Iout,M6020_MOTOR_POSITION_PID_MAX_IOUT);
    //KD:
    pid->Dout=pid->KD*(pid->error[0]-2*pid->error[1]+pid->error[2]);

    pid->out+=pid->Pout+pid->Iout+pid->Dout;
    pid->out=LimitMax(pid->out,M6020_MOTOR_POSITION_PID_MAX_OUT);

		pid->error[2]=pid->error[1];
    pid->error[1]=pid->error[0];
		
    return pid->out;
}

/*功能：PID初始化函数
参数：pid（pid的类型:速度环还是角度环）
    PID[3]：PID参数
    max_out 最大输出
    max_iout 最大积分输出
返回：无
*/
void PID_Init(PID_type_def *pid,const float PID[3],float max_out,float max_iout)
{
    if(pid==0||PID==0)
    {
        return;
    }

    pid->KP=PID[0];
    pid->KI=PID[1];
    pid->KD=PID[2];
    pid->max_out=max_out;
    pid->max_Iout=max_iout;
    pid->error[0]=pid->error[1]=pid->error[2]=pid->Pout=pid->Iout=pid->Dout=pid->out=0.0f;
}

/*功能：MOTOR的PID初始化函数
参数：motor6020:哪个电机
返回：无
*/
void MOTOR_Init(MOTOR_t *motor6020)
{
// 		const float motor_6020_position_pid_argument[3]={M6020_MOTOR_POSITION_PID_KP,M6020_MOTOR_POSITION_PID_KI,M6020_MOTOR_POSITION_PID_KD};//位置环
//    PID_Init(&motor6020->position_PID,motor_6020_position_pid_argument,M6020_MOTOR_POSITION_PID_MAX_OUT,M6020_MOTOR_POSITION_PID_MAX_IOUT);
	
		const float motor_6020_speed_pid_argument[3]={M6020_MOTOR_SPEED_PID_KP,M6020_MOTOR_SPEED_PID_KI,M6020_MOTOR_SPEED_PID_KD};//速度环
    PID_Init(&motor6020->speed_PID,motor_6020_speed_pid_argument,M6020_MOTOR_POSITION_PID_MAX_OUT,M6020_MOTOR_POSITION_PID_MAX_IOUT);
}
