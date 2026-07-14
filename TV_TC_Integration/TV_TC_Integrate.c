#include"TC.h"
#include"TV.h"
#include"TV_TC_Integrate.h"
#include<math.h>
#include<stdio.h>

WheelTorque torque_manager(
        float driver_torque,
        float steer_angle,
        float vehicle_speed,
        float yaw_rate,
        int rpm_left,
        int rpm_right)
{
    WheelTorque out;

    float target_yaw =
        Target_yaw_rate(
            steer_angle,
            vehicle_speed);

    float kp = get_kp(vehicle_speed);
    float ki = get_ki(vehicle_speed);

    float u =
        pid_controller(
            target_yaw,
            yaw_rate,
            kp,
            ki);

    float tv_left =
        torque_outside(
            u,
            driver_torque);

    float tv_right =
        torque_inside(
            u,
            driver_torque);

    float tc_left =
        tc_limit(
            vehicle_speed,
            rpm_left);

    float tc_right =
        tc_limit(
            vehicle_speed,
            rpm_right);

    out.left =
        fminf(tv_left,
              tc_left);

    out.right =
        fminf(tv_right,
              tc_right);

    return out;
}
