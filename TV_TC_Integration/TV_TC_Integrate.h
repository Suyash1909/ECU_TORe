#ifndef TV_TC_INTEGRATE
#define TV_TC_INTEGRATE
typedef struct
{
    float left;
    float right;
} WheelTorque;

WheelTorque torque_manager(
        float driver_torque,
        float steer_angle,
        float vehicle_speed,
        float yaw_rate,
        int rpm_left,
        int rpm_right);

#endif
