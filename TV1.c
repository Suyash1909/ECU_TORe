#include "TV.h"
#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>

#include <stdio.h>


#define MASS        270.0
#define MAX_TORQUE  29.1

/* Vehicle Geometry */
#define WHEELBASE   1.532
#define Lr          (WHEELBASE * 0.55)          // 0.8426 m
#define Lf          (WHEELBASE - Lr)             // 0.6894 m
#define trackwidth  1.235

/* Tyre Cornering Stiffness */
#define Cf          24002.745
#define Cr          28354.469

/* Vehicle Parameters */
#define Radius      0.2286
#define gear_ratio  10.05
#define Izz         47.563

/* Environment */
#define U           1.6          // Friction coefficient used in TV controller
#define g           9.81
#define rho         1.225
#define Cd          0.7
#define Area        0.99
#define COG_HEIGHT  0.201

/* Rolling Resistance */
#define MU_ROLLING  0.015

/* Front Rolling Resistance Force */
#define Fx_f        (MU_ROLLING * MASS * g)

#define VEL_TAB_SIZE 8
#define KU           0.001902

float Kp_table[8]= {-2239.7830f,8876.3383, 11354.2364, 12510.5888, 13180.0560, 13616.6651, 13923.9085, 14151.8633};
float Ki_table[8]= {-410137.6804f,308766.9421, 371457.8691, 423444.9439, 460154.3236, 486707.3677, 506629.0431, 522069.5479};
double Vx_table[8]={3.0f,7.0f,11.0f,15.0f,19.0,23.0f,27.0f,31.0f};




// double Understeer_gradient_calc(void){
//     return (((Lr*MASS)/(Cf*WHEELBASE))-((Lf*MASS)/(Cr*WHEELBASE)));
// }

double Target_yaw_rate(double steer_angle, double Vx){
    // float Ku=Understeer_gradient_calc();
    
    // Prevent division by zero at vehicle launch
    if (Vx < 0.001) {
        Vx = 0.001;
    }

    double yaw_target=(((steer_angle*Vx)/(WHEELBASE+(KU*Vx*Vx))));
    if(yaw_target>((U*g)/Vx)){
        yaw_target=(U*g)/Vx;
    }
    if(yaw_target<(-(U*g)/Vx)){
        yaw_target=-(U*g)/Vx;
    }
    return yaw_target;
}

int binary_search(const double arr[], int size, double x ){
    int low =0;
    int high=size-1;

    while((high-low)>1){
        int mid=(low+high)/2;
       
        if(x>=arr[mid]){
            low=mid;
        }
        else {
            high=mid;
        }
    }
    return low;
}

double get_kp(double Vx){
    double x1, x2;
    double y1, y2;
    
    if(Vx < Vx_table[0]){
        Vx=Vx_table[0];
    }
    if(Vx > Vx_table[VEL_TAB_SIZE-1]){
        Vx=Vx_table[VEL_TAB_SIZE-1];
    }

    int i = binary_search(Vx_table, VEL_TAB_SIZE, Vx);
    x1 = Vx_table[i];
    x2 = Vx_table[i+1];
    y1 = Kp_table[i];
    y2 = Kp_table[i+1];

    double kp = y1 + ((Vx - x1) * (y2 - y1)) / (x2 - x1);
    return kp;
}

double get_ki(double Vx){
    double x1, x2;
    double y1, y2;
    
    if(Vx < Vx_table[0]){
        Vx=Vx_table[0];
    }
    if(Vx > Vx_table[VEL_TAB_SIZE-1]){
        Vx=Vx_table[VEL_TAB_SIZE-1];
    }

    int i = binary_search(Vx_table, VEL_TAB_SIZE, Vx);
    x1 = Vx_table[i];
    x2 = Vx_table[i+1];
    y1 = Ki_table[i];
    y2 = Ki_table[i+1];

    double ki = y1 + ((Vx - x1) * (y2 - y1)) / (x2 - x1);
    return ki;
}

double pid_controller(double ref, double y, double Kp, double Ki, double *integral_state)
{
    double u_max = (MAX_TORQUE * 10.05 / Radius) * trackwidth * 0.5;
    double u_min = -u_max;
    double T = 0.001;
    double error = ref - y;
    double P = Kp * error;
    double I = Ki * (*integral_state);

     double u_unsat = P + I;
    double u;

    if (u_unsat > u_max){u = u_max;}
    else if (u_unsat < u_min){u = u_min;}
    else {u = u_unsat;
        *integral_state += T * error; }

    return u;
}

double torque_dist(double u){
    double delta_t=(u)*(Radius/trackwidth);
    return delta_t;
}

double torque_left(double delta_t,double torque_current_left,double Vx){
  if(Vx<3.0f){
        delta_t=0.0f;
    }
   double tout=torque_current_left-delta_t;

   return tout;
} 

double torque_right(double delta_t,double torque_current_right,double Vx){
    if(Vx<3.0f){
        delta_t=0.0f;
    }
    
  double tin=torque_current_right+delta_t;


   return tin;
}

Torque final_dist_wrapper(double current_yawrate,double delta,double Vx,double *integral_state,double torque_demand_raw){
    double torque_demand=torque_demand_raw*gear_ratio;
    double target_yaw=Target_yaw_rate(delta,Vx);
    double p=get_kp(Vx);
    double i=get_ki(Vx);
    double req_yaw_additional=pid_controller(target_yaw,current_yawrate,p,i,integral_state);
    double torque_delta=torque_dist(req_yaw_additional);
    double tor_right=torque_right(torque_delta,torque_demand,Vx);
    double tor_left=torque_left(torque_delta,torque_demand,Vx);
      Torque T;
  T.right = tor_right / gear_ratio;
  T.left  = tor_left / gear_ratio;
  return T;

//     printf("Target yaw = %f\n", target_yaw);
// printf("Kp = %f\n", p);
// printf("Ki = %f\n", i);
// printf("PID output = %f\n", req_yaw_additional);
// printf("Torque delta = %f\n", torque_delta);
// printf("Left torque = %f\n", tor_left);
// printf("Right torque = %f\n", tor_right);

}

// double final_dist_wapper_left(double current_yawrate,double delta,double Vx,double Ku,double *integral_state,double torque_demand){
//     double target_yaw=Target_yaw_rate(delta,Vx,Ku);
//     double p=get_kp(Vx);
//     double i=get_ki(Vx);
//     double req_yaw_additional=pid_controller(target_yaw,current_yawrate,p,i,integral_state);
//     double torque_delta=torque_dist(req_yaw_additional);
//     double tor_right=torque_right(torque_delta,torque_demand,Vx);
//     double tor_left=torque_left(torque_delta,torque_demand,Vx);
//     return tor_left;

// }
// double final_dist_wapper_right(double current_yawrate,double delta,double Vx,double Ku,double *integral_state,double torque_demand){
//     double target_yaw=Target_yaw_rate(delta,Vx,Ku);
//     double p=get_kp(Vx);
//     double i=get_ki(Vx);
//     double req_yaw_additional=pid_controller(target_yaw,current_yawrate,p,i,integral_state);
//     double torque_delta=torque_dist(req_yaw_additional);
//     double tor_right=torque_right(torque_delta,torque_demand,Vx);
//     double tor_left=torque_left(torque_delta,torque_demand,Vx);
//     return tor_right;

// }