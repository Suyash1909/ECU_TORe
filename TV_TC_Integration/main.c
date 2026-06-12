#include<stdio.h>
#include"TV_TC_Integrate.h"

int main(void)
{
    WheelTorque wheel_torque;

//         float driver_torque,
//         float steer_angle,
//         float vehicle_speed,
//         float yaw_rate,
//         int rpm_left,
//         int rpm_right
    wheel_torque =
        torque_manager(
            20.0f,      
            0.1f,       
            7.0f,       
            0.0f,       
            3000,       
            2900);      

    printf("Left  = %f\n",
            wheel_torque.left);

    printf("Right = %f\n",
            wheel_torque.right);
}