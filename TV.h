#ifndef TV_H
#define TV_H
typedef struct TV1
{
    double left;
    double right;
}Torque;

double get_kp(double Vx);
double get_ki(double Vx);
// double Understeer_gradient_calc(void);
double Target_yaw_rate(double steer_angle, double Vx);
double pid_controller(double ref, double y, double Kp, double Ki, double *integral_state);
double torque_right(double delta_t,double torque_current_right,double Vx);
double torque_left(double delta_t,double torque_current_left,double Vx);
Torque final_dist_wrapper(double current_yawrate,double delta,double Vx,double *integral_state,double torque_demand_raw);
double torque_dist(double u);

// double final_dist_wapper_left(double current_yawrate,double delta,double Vx,double Ku,double *integral_state,double torque_demand);
// double final_dist_wapper_right(double current_yawrate,double delta,double Vx,double Ku,double *integral_state,double torque_demand);
#endif