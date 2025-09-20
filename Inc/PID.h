
#ifndef PID_H_
#define PID_H_

#include "stdint.h"
typedef int16_t motor_measure;//改个名字

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

/*功能：PID速度环输出值的计算
参数:pid（哪个环）
now_value:现在的值
set_value:设定值
返回：PID输出的结果
*/
float PID_Speed_Calculate(PID_type_def *pid,float now_value,float set_value);
/*功能：PID输出值的计算
参数:pid（哪个环）
now_value:现在的值
set_value:设定值
返回：PID输出的结果
*/
float PID_Position_Calculate(PID_type_def *pid,float now_value,float set_value);

/*功能：PID初始化函数
参数：pid（pid的类型:速度环还是角度环）
    PID[3]：PID参数
    max_out 最大输出
    max_iout 最大积分输出
返回：无
*/
void PID_Init(PID_type_def *pid,const float PID[3],float max_out,float max_iout);

/*功能：MOTOR的PID初始化函数
参数：motor6020:哪个电机
返回：无
*/
void MOTOR_Init(MOTOR_t *motor6020);
#endif
