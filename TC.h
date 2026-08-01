#ifndef TC_H
#define TC_H
double omega_cal(double motor_rpm);
double lambda_cal(double omega,double velocity);
double torque_command(double velocity, double lambda, double vectored_torque_raw);
// float tc_limit(float velocity,int motor_rpm);
double get_Kp_val(double velocity, double lambda);
double get_ki_val(double velocity, double lambda);
#endif