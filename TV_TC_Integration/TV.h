#ifndef TV_H
#define TV_H

float Target_yaw_rate(float steer_angle, float Vx);
float get_kp(float Vx);
float get_ki(float Vx);
double pid_controller(double ref,double y, double kp, double ki
);
float torque_inside(float u,float torque_current_right);
float torque_outside(float u,float torque_current_left);
#endif