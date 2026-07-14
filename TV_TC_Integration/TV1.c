#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>

#define MASS 270
#define MAX_TORQUE 29.1f
#define Lf 0.78132f  //1.532*0.51
#define Lr 0.75068  
#define WHEELBASE (Lf+Lr)
#define trackwidth 1.235
#define Cf 35238.459f
#define Cr 35033.582f
#define U 1.2f   //friction coeff
#define g 9.81
#define Radius 0.2286f
#define VEL_TAB_SIZE 21
#define gear_ratio 10.05

float Kp_table[21]= {-37942.0000f, -11126.0000f, -2187.0000f, 2282.0000f, 4964.0000f, 6752.0000f, 8029.0000f, 8986.0000f, 9731.0000f, 
    10327.0000f, 10815.0000f, 11221.0000f, 11565.0000f, 11860.0000f, 12115.0000f, 12339.0000f, 12536.0000f, 12711.0000f, 12868.0000f, 13009.0000f, 13137.0000f};
float Ki_table[21]= {-263530.0000f, -642290.0000f, -409930.0000f, -226510.0000f, -94940.0000f, 1730.0000f, 75180.0000f, 132670.0000f, 178800.0000f, 216600.0000f, 
    248130.0000f, 274800.0000f, 297670.0000f, 317470.0000f, 334800.0000f, 350080.0000f, 363650.0000f, 375790.0000f, 386710.0000f, 396580.0000f, 405560.0000f};
float Vx_table[21]={1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f, 18.0f, 19.0f, 20.0f, 21.0f};



float Understeer_gradient_clac(void){
    return (((Lr*MASS)/(Cf*WHEELBASE))-((Lf*MASS)/(Cr*WHEELBASE)));
    

}

float Target_yaw_rate(float steer_angle, float Vx){
    float Ku=Understeer_gradient_clac();

    float yaw_target=(((steer_angle*Vx)/(WHEELBASE+(Ku*Vx*Vx))));
    if(yaw_target>((U*g)/Vx)){
        yaw_target=(U*g)/Vx;
    }
    if(yaw_target<(-(U*g)/Vx)){
        yaw_target=-(U*g)/Vx;
    }
    return yaw_target;


}
int binary_search(float arr[],int size,float x ){
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


float get_kp(float Vx){
    float x1;
    float x2;
    float y1;
    float y2;
     if(Vx< Vx_table[0]){
        Vx=Vx_table[0];
    }
    if(Vx>Vx_table[VEL_TAB_SIZE-1]){
        Vx=Vx_table[VEL_TAB_SIZE-1];
    }

 int i=binary_search(Vx_table, VEL_TAB_SIZE,Vx);
 x1=Vx_table[i];
 x2=Vx_table[i+1];
 y1=Kp_table[i];
 y2=Kp_table[i+1];

 float kp=y1+((Vx-x1)*(y2-y1))/(x2-x1);
 return kp;
}

float get_ki(float Vx){
    float x1;
    float x2;
    float y1;
    float y2;
     if(Vx< Vx_table[0]){
        Vx=Vx_table[0];
    }
    if(Vx>Vx_table[VEL_TAB_SIZE-1]){
        Vx=Vx_table[VEL_TAB_SIZE-1];
    }

 int i=binary_search(Vx_table, VEL_TAB_SIZE,Vx);
 x1=Vx_table[i];
 x2=Vx_table[i+1];
 y1=Ki_table[i];
 y2=Ki_table[i+1];

 float ki=y1+((Vx-x1)*(y2-y1))/(x2-x1);
 return ki;
}

double pid_controller(double ref, double y, double Kp, double Ki)
{
    static double integral = 0.0;

    const double T = 0.001;

    double u_max = (MAX_TORQUE * 10.05 / 0.2286) * 1.235;
    double u_min = -u_max;

    double error = ref - y;

    double P = Kp * error;
    double I = Ki * integral;

    double u_unsat = P + I;
    double u;

    if (u_unsat > u_max)
    {
        u = u_max;
    }
    else if (u_unsat < u_min)
    {
        u = u_min;
    }
    else
    {
        u = u_unsat;

        /* Anti-windup */
        integral += 0.5 * T * error;
    }

    return u;
}

float torque_dist(float u){
    float delta_t=(u*Radius)/trackwidth;
    return delta_t;

}
float torque_outside(float u,float torque_current_left){
    float delta_t=torque_dist(u);
   float tout=torque_current_left+(delta_t/10.05f);
   if(tout<0){
    tout=0;
   }
   if(tout>29.1f){
    tout=29.1;   }
    return tout;

}
float torque_inside(float u,float torque_current_right){
    float delta_t=torque_dist(u);
   float tin=torque_current_right-(delta_t/10.05f);
   if(tin<0){
    tin=0;
   }
   if(tin>29.1f){
    tin=29.1;   }
    return tin;

}







// int main(){
//     float velocity=7.0f;
//     float steering_angle=-0.1f;
//     float current_torque_left=0.0f;
//     float current_torque_right=0.0f;
//     float current_yaw_rate= 0.0f;
//     float under_steer_grad=Understeer_gradient_clac();
//     float target_yaw=Target_yaw_rate(steering_angle,velocity);
//     float ki=get_ki(velocity);
//     float kp=get_kp(velocity);
//     double u=pid_controller(target_yaw,current_yaw_rate,kp,ki);
//     float tout=torque_outside(u,current_torque_left);
//     float tin=torque_inside(u,current_torque_right);

//     printf("under_steer_grad %f ",under_steer_grad);
//     printf(" %f \n",target_yaw);
//     printf(" %f \n",ki);
//     printf(" %f \n",kp);
//     printf(" %f \n",u);
//     printf(" %f \n",tin);
//     printf(" %f \n",tout);

// }
