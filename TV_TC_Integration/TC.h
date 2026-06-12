#ifndef TC_H
#define TC_H
float omega_cal(int motor_rpm);
float lambda_cal(float omega,float velocity);
float torque_command(float velocity,float lambda);
float tc_limit(float velocity,int motor_rpm);
#endif