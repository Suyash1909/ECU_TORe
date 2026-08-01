#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include"TV.h"
#include "TC.h"
#define LAMBDA_DESIRED 0.08f
#define PI 3.1415926f
#define GEAR_RATIO 10.05f
#define RADIUS 0.2286f
#define VEL_TAB_SIZE 5
#define LAMBDA_TAB_SIZE 9
#define MAX_TORQUE 29.1f

double velocity_Table[5]={0.1, 2.5, 5.0, 7.5, 10.0};
double lambda_table[9]={0.1, 0.13, 0.15, 0.2, 0.25, 0.4, 0.6, 0.8, 0.990};

double Kp_Table[5][9]={
    {0,0, 0, 0, 0, 0, 0, -27.349359727984901, -16.0066270143361},
    {-12.762924193853999, -13.379408025363601,-13.8128407190821, -15.345193573942,-20.357279305276101, -28.198949446354199, -32.052075584671499, -34.566878493769799, 0},
    {-22.9504856800482, -24.700528926158601, -26.420442641173398, -32.7735727935966, -37.1302685077357, -47.340341183748301, 0, 0, 0},
    {-28.7804511556023, -29.824471630540199, -32.138751342701099, -37.855783576526598, -42.107584251118702, -55.059013875800197, 0, 0, 0},
    {-39.576096959295903, -49.687581205460198, -60.003709338410196, -77.405667412310194, -87.476305087217199, 0, 0, 0, 0}
};

double Ki_Table[5][9]={
    {0, 0, 0, 0, 0, 0,0, -11670.790945277, -11739.8305053856},
    {-561.66596120840995,-556.78905522505704, -554.70793302180698, -574.58960009903603, -580.0, -591.63143190700202, -609.58566806772399, -657.67272906434596,0},
    {-457.10946338920201,-437.465599128539, -629.40613477025499, -758.94918952921705, -822.20916479232903, -907.77754639936097, 0, 0, 0},
    {-557.75879260435795, -508.84249924102602, -540.07337548326495, -613.94832258016299, -651.92334489266898, -755.40834234298904, 0, 0, 0},
    {-436.84711841410899,-567.02082103832402, -709.61936431326399, -966.68353937852203, -1165.13113408405, 0, 0, 0, 0}
};

double lambda_cal(double omega, double velocity){
    if(fabs(omega) < 0.00001 ){
        return 0.0;
    }
    double lambda = ((omega * RADIUS) - velocity) / (omega * RADIUS);
    return lambda;
}

double omega_cal(double motor_rpm){
    double omega = (motor_rpm * 2 * PI) / (GEAR_RATIO * 60);
    return omega;
}

double linear_interpoletion(double x0, double x1, double y0, double y1, double x){
    return y0 + ((x - x0) * (y1 - y0)) / (x1 - x0);
}

// FIXED: Upgraded to accept double arrays
int binary_Search(const double arr[], int size, double x ){
    int low = 0;
    int high = size - 1;

    while((high - low) > 1){
        int mid = (low + high) / 2;
        if(x >= arr[mid]){
            low = mid;
        } else {
            high = mid;
        }
    }
    return low;
}

double get_Kp_val(double velocity, double lambda){
    if(velocity < velocity_Table[0]){
        velocity = velocity_Table[0];
    }
    if(velocity > velocity_Table[VEL_TAB_SIZE-1]){
        velocity = velocity_Table[VEL_TAB_SIZE-1];
    }
    if(lambda < lambda_table[0]){
        lambda = lambda_table[0];
    }
    if(lambda > lambda_table[LAMBDA_TAB_SIZE-1]){
        lambda = lambda_table[LAMBDA_TAB_SIZE-1];
    }

    int i = binary_Search(velocity_Table, VEL_TAB_SIZE, velocity);
    int j = binary_Search(lambda_table, LAMBDA_TAB_SIZE, lambda);

    double Q11 = Kp_Table[i][j];
    double Q12 = Kp_Table[i+1][j];
    double Q21 = Kp_Table[i][j+1];
    double Q22 = Kp_Table[i+1][j+1];

    double R1 = linear_interpoletion(velocity_Table[i], velocity_Table[i+1], Q11, Q12, velocity);
    double R2 = linear_interpoletion(velocity_Table[i], velocity_Table[i+1], Q21, Q22, velocity);
    double P = linear_interpoletion(lambda_table[j], lambda_table[j+1], R1, R2, lambda);

    return P;
}

double get_ki_val(double velocity, double lambda){
    if(velocity < velocity_Table[0]){
        velocity = velocity_Table[0];
    }
    if(velocity > velocity_Table[VEL_TAB_SIZE-1]){
        velocity = velocity_Table[VEL_TAB_SIZE-1];
    }
    if(lambda < lambda_table[0]){
        lambda = lambda_table[0];
    }
    if(lambda > lambda_table[LAMBDA_TAB_SIZE-1]){
        lambda = lambda_table[LAMBDA_TAB_SIZE-1];
    }

    int i = binary_Search(velocity_Table, VEL_TAB_SIZE, velocity);
    int j = binary_Search(lambda_table, LAMBDA_TAB_SIZE, lambda);

    double Q11 = Ki_Table[i][j];
    double Q12 = Ki_Table[i+1][j];
    double Q21 = Ki_Table[i][j+1];
    double Q22 = Ki_Table[i+1][j+1];

    double R1 = linear_interpoletion(velocity_Table[i], velocity_Table[i+1], Q11, Q12, velocity);
    double R2 = linear_interpoletion(velocity_Table[i], velocity_Table[i+1], Q21, Q22, velocity);
    double I = linear_interpoletion(lambda_table[j], lambda_table[j+1], R1, R2, lambda);

    return I;
}
double torque_command(double velocity, double lambda, double vectored_torque_raw){
    // printf("Entered torque_command()\n");
    double vectored_torque=vectored_torque_raw*GEAR_RATIO;
    double error_lambda = LAMBDA_DESIRED - lambda;
    
    double P = get_Kp_val(velocity, lambda) * GEAR_RATIO;
    
    double torque_to_be_reduced = (error_lambda * P); 
    
    if(torque_to_be_reduced < 0){
        torque_to_be_reduced = 0;
    }
    
    if(torque_to_be_reduced > (MAX_TORQUE * GEAR_RATIO)){
        torque_to_be_reduced = (MAX_TORQUE * GEAR_RATIO);
    }
    if((vectored_torque - torque_to_be_reduced)>(MAX_TORQUE*GEAR_RATIO)){
        return MAX_TORQUE;
    }
    if((vectored_torque - torque_to_be_reduced)<0){
        return 0; 
    }
//     printf("error_lambda = %f\n", error_lambda);
// printf("P = %f\n", P);
// printf("torque_to_be_reduced = %f\n", torque_to_be_reduced);
// printf("returning = %f\n", vectored_torque - torque_to_be_reduced);
    return (vectored_torque - torque_to_be_reduced)/GEAR_RATIO;


}


// double torque_command_left(double velocity, double lambda,double vectored_torque_left){
//     double error_lambda = LAMBDA_DESIRED - lambda;
//     double P = get_Kp_val(velocity, lambda) * GEAR_RATIO;
    
//     double torque_to_be_reduced = (error_lambda * P); 
    
//     if(torque_to_be_reduced < 0){
//         torque_to_be_reduced = 0;
//     }
    
//     if(torque_to_be_reduced > (MAX_TORQUE * GEAR_RATIO)){
//         torque_to_be_reduced = (MAX_TORQUE * GEAR_RATIO);
//     }
//     return vectored_torque_left - torque_to_be_reduced;
// }
// double torque_command_right(double velocity, double lambda, double vectored_torque_right){
//     double error_lambda = LAMBDA_DESIRED - lambda;
//     double P = get_Kp_val(velocity, lambda) * GEAR_RATIO;
    
//     double torque_to_be_reduced = (error_lambda * P); 
    
//     if(torque_to_be_reduced < 0){
//         torque_to_be_reduced = 0;
//     }
    
//     if(torque_to_be_reduced > (MAX_TORQUE * GEAR_RATIO)){
//         torque_to_be_reduced = (MAX_TORQUE * GEAR_RATIO);
//     }
//     return vectored_torque_right - torque_to_be_reduced;
// }
