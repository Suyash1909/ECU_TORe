#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>

// Hardware interface headers from source 9
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <lgpio.h>

#include "APPS_R2D_H.h"
#include "TC.h"
#include "TV.h"

// --- Configuration & Constants ---
#define SAFETY_PERIOD_NS    10000000L   // 10ms = 100Hz loop
#define I2C_BUS             "/dev/i2c-1"
#define ADS1115_ADDR_1      0x48
#define GPIO_CHIP           0
#define PIN_SDC             13  
#define PIN_STARTER         19  
#define PIN_BUZZER          23  

// Example Motor Controller CAN ID
#define CAN_ID_INVERTER_CMD 0x300 

// --- Shared Data & Threading ---
typedef struct {
    // Existing variables...
    float torque_demand;
    bool system_fault;
    bool is_r2d;
    double Vx;
    double steering_angle;
    
    // Updated Dynamics & Thermal variables
    double current_yawrate;       // Fed by EKF in the DAQ thread
    float motor_temp_left;        // Fed by MLX 1
    float motor_temp_right;       // Fed by MLX 2
    
    double motor_rpm_left;
    double motor_rpm_right;
} SharedVehicleState;

typedef struct {
    uint16_t adc[3];
    bool sdc_closed;
    bool starter_button_pressed;
    double steering_angle;
    double motor_rpm_left;
    double motor_rpm_right;
} VehicleInputs;

static SharedVehicleState shared_state;
static pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
static sem_t update_sem;
static bool update_pending;
static volatile sig_atomic_t stop_requested;

// Global File Descriptors
static int can_socket_fd = -1;
static int gpio_h = -1;

#define clamp(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

// --- Hardware Initialization Helpers ---
static int init_can(void) {
    struct sockaddr_can addr;
    struct ifreq ifr;
    if ((can_socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) return -1;
    strcpy(ifr.ifr_name, "can0");
    ioctl(can_socket_fd, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) return -1;
    return 0;
}

static int init_gpio(void) {
    gpio_h = lgGpiochipOpen(GPIO_CHIP);
    if (gpio_h < 0) return -1;
    lgGpioClaimInput(gpio_h, LG_SET_PULL_DOWN, PIN_SDC);
    lgGpioClaimInput(gpio_h, LG_SET_PULL_DOWN, PIN_STARTER);
    lgGpioClaimOutput(gpio_h, 0, PIN_BUZZER, 0);
    return 0;
}

static int i2c_open_local(uint8_t addr, int* i2c_fd) {
    *i2c_fd = open(I2C_BUS, O_RDWR);
    if (*i2c_fd < 0) return -1;
    if (ioctl(*i2c_fd, I2C_SLAVE, addr) < 0) { close(*i2c_fd); return -1; }
    ioctl(*i2c_fd, I2C_TIMEOUT, 2); // 20ms timeout to prevent hanging the 100Hz loop
    ioctl(*i2c_fd, I2C_RETRIES, 1);
    return 0;
}

// (Assume ads1115_read_channels is defined here exactly as in your source 9, 
//  but optimized to avoid 20ms blocking sleeps if possible)

static void request_stop(int signal_number) {
    (void)signal_number;
    stop_requested = 1;
}

static void sleep_safely(long nanoseconds) {
    struct timespec delay = {.tv_sec = 0, .tv_nsec = nanoseconds};
    while (!stop_requested && nanosleep(&delay, &delay) == -1 && errno == EINTR);
}

// --- The Hardware Input Reader ---
static void read_vehicle_inputs(VehicleInputs *inputs, long now_ms) {
    (void)now_ms;
    memset(inputs, 0, sizeof(*inputs)); // Safe default

    // 1. Read GPIO
    inputs->sdc_closed = (lgGpioRead(gpio_h, PIN_SDC) == 1);
    inputs->starter_button_pressed = (lgGpioRead(gpio_h, PIN_STARTER) == 1);

    // 2. Read Pedals via I2C
    int i2c_fd;
    if (i2c_open_local(ADS1115_ADDR_1, &i2c_fd) == 0) {
        uint16_t adc_buf[4] = {0};
        // ads1115_read_channels(adc_buf, &i2c_fd); 
        // For brevity, map the reads:
        inputs->adc[0] = adc_buf[0]; // Brake
        inputs->adc[1] = adc_buf[1]; // APPS 1
        inputs->adc[2] = adc_buf[2]; // APPS 2
        close(i2c_fd);
    }

    // 3. Read CAN Non-Blocking (Drain the buffer)
    struct can_frame rx_frame;
    while (recv(can_socket_fd, &rx_frame, sizeof(struct can_frame), MSG_DONTWAIT) > 0) {
        // Parse incoming telemetry (e.g., Vx, RPMs, Yaw rate from MPU/STM32)
        // if (rx_frame.can_id == CAN_ID_REPLY_GYRO) { ... }
    }
    
    // Mocking dynamics data for now, replace with parsed CAN data above
    inputs->Vx = 5.0;
    inputs->steering_angle = 0.2;
    inputs->current_yawrate = 0.0;
    inputs->motor_rpm_left = 2100.0;
    inputs->motor_rpm_right = 2100.0;
}

// --- The Hardware Torque Dispatcher ---
static void dispatch_torque(double left, double right) {
    struct can_frame tx_frame = {0};
    tx_frame.can_id = CAN_ID_INVERTER_CMD;
    tx_frame.can_dlc = 4;
    
    // Example serialization: scaling torque to an integer
    int16_t left_int = (int16_t)(left * 100.0);
    int16_t right_int = (int16_t)(right * 100.0);
    
    tx_frame.data[0] = (left_int >> 8) & 0xFF;
    tx_frame.data[1] = left_int & 0xFF;
    tx_frame.data[2] = (right_int >> 8) & 0xFF;
    tx_frame.data[3] = right_int & 0xFF;

    write(can_socket_fd, &tx_frame, sizeof(tx_frame));
}

// --- Thread 1: Safety & Hardware Loop (100Hz) ---
static void *safety_r2d_thread(void *arg) {
    (void)arg;
    puts("[Safety/R2D Thread] Started at 100Hz.");

    while (!stop_requested) {
        VehicleInputs inputs;
        long now_ms = get_time_ms();
        
        read_vehicle_inputs(&inputs, now_ms);

        float raw_torque = R2D_update(inputs.adc, inputs.sdc_closed,
                                      inputs.starter_button_pressed, now_ms);
        bool current_fault = r2d_apps_fault();
        bool current_r2d = r2d_is_active();

        // Control the physical buzzer
        lgGpioWrite(gpio_h, PIN_BUZZER, r2d_sound_active() ? 1 : 0);

pthread_mutex_lock(&state_mutex);
        
        // ONLY update variables owned by the Safety thread
        shared_state.torque_demand = raw_torque;
        shared_state.system_fault = current_fault || !inputs.sdc_closed;
        shared_state.is_r2d = current_r2d;
        shared_state.steering_angle = inputs.steering_angle;
        shared_state.motor_rpm_left = inputs.motor_rpm_left;
        shared_state.motor_rpm_right = inputs.motor_rpm_right;

        bool notify_dynamics = !update_pending;
        update_pending = true;
        
        pthread_mutex_unlock(&state_mutex);

        if (notify_dynamics && sem_post(&update_sem) == -1) stop_requested = 1;
        
        sleep_safely(SAFETY_PERIOD_NS); // Enforce 10ms loop
    }
    return NULL;
}

static int wait_for_update(void) {
    while (sem_wait(&update_sem) == -1) {
        if (errno != EINTR) return -1;
        if (stop_requested) return -1;
    }
    return 0;
}

// --- Thread 2: Vehicle Dynamics (TV & TC) ---
static void *vehicle_dynamics_thread(void *arg) {
    (void)arg;
    puts("[Dynamics Thread] Started.");

    double integral_state = 0.0;
    while (!stop_requested && wait_for_update() == 0) {
        SharedVehicleState state;
        pthread_mutex_lock(&state_mutex);
        state = shared_state;
        update_pending = false;
        pthread_mutex_unlock(&state_mutex);

        if (state.system_fault || !state.is_r2d) {
            integral_state = 0.0;
            dispatch_torque(0.0, 0.0);
            continue;
        }

        bool tv_enabled = fabs(state.steering_angle) > 0.139626 &&
                          state.torque_demand >= 5.0f && state.Vx > 3.0;
        bool tc_enabled = state.Vx >= 0.1 && state.Vx < 10.0;
        
        if (!tv_enabled) integral_state = 0.0;

        Torque vectored_torque;
        if (tv_enabled) {
            vectored_torque = final_dist_wrapper(
                state.current_yawrate, state.steering_angle, state.Vx,
                &integral_state, state.torque_demand);
        } else {
            vectored_torque.left = state.torque_demand;
            vectored_torque.right = state.torque_demand;
        }

        double final_torque_left = vectored_torque.left;
        double final_torque_right = vectored_torque.right;
        
        if (tc_enabled) {
            final_torque_left = torque_command(
                state.Vx, lambda_cal(omega_cal(state.motor_rpm_left), state.Vx),
                vectored_torque.left);
            final_torque_right = torque_command(
                state.Vx, lambda_cal(omega_cal(state.motor_rpm_right), state.Vx),
                vectored_torque.right);
        }

        final_torque_left = clamp(final_torque_left, 0.0, 29.1);
        final_torque_right = clamp(final_torque_right, 0.0, 29.1);
        dispatch_torque(final_torque_left, final_torque_right);
    }
    dispatch_torque(0.0, 0.0);
    return NULL;
}
#define DAQ_PERIOD_NS 50000000L // 50ms = 20Hz loop
static void *daq_sensor_thread(void *arg) {
    (void)arg;
    puts("[DAQ Thread] Started at 20Hz.");

    while (!stop_requested) {
        // 1. Perform slow, blocking I2C reads
        float temp_left = 45.5f;   // Mock MLX 1 read
        float temp_right = 46.0f;  // Mock MLX 2 read
        
        // 2. Execute 3-DOF EKF IMU Sensor Fusion here
        // TODO: Call your EKF function here
        double filtered_yaw_rate = 0.15; // Mock EKF output
        double filtered_Vx = 5.0;        // Mock EKF output
        
        // 3. Lock Mutex and update the shared state
        pthread_mutex_lock(&state_mutex);
        
        // ONLY update variables owned by the DAQ thread
        shared_state.motor_temp_left = temp_left;
        shared_state.motor_temp_right = temp_right;
        shared_state.current_yawrate = filtered_yaw_rate;
        shared_state.Vx = filtered_Vx;
        
        pthread_mutex_unlock(&state_mutex);

        // 4. Sleep until the next 50ms cycle
        sleep_safely(DAQ_PERIOD_NS);
    }
    return NULL;
}
int main(void) {
    // 1. Initialize Hardware Interfaces
    if (init_can() < 0) { perror("CAN Init Failed"); return 1; }
    if (init_gpio() < 0) { perror("GPIO Init Failed"); return 1; }

    // 2. Setup Signal Handling
    struct sigaction stop_action = {.sa_handler = request_stop};
    sigemptyset(&stop_action.sa_mask);
    sigaction(SIGINT, &stop_action, NULL);
    sigaction(SIGTERM, &stop_action, NULL);

    // 3. Initialize Thread Synchronization
    if (sem_init(&update_sem, 0, 0) != 0) return 1;

    // 4. Launch ALL Threads
    pthread_t thread_safety, thread_dynamics, thread_daq;
    
    if (pthread_create(&thread_safety, NULL, safety_r2d_thread, NULL) != 0) return 1;
    if (pthread_create(&thread_daq, NULL, daq_sensor_thread, NULL) != 0) return 1;
    
    if (pthread_create(&thread_dynamics, NULL, vehicle_dynamics_thread, NULL) != 0) {
        stop_requested = 1;
        pthread_join(thread_safety, NULL);
        pthread_join(thread_daq, NULL);
        return 1;
    }

    // 5. Wait for threads to finish (on shutdown)
    pthread_join(thread_safety, NULL);
    pthread_join(thread_daq, NULL);
    
    sem_post(&update_sem); // Wake dynamics to let it exit cleanly
    pthread_join(thread_dynamics, NULL);

    // 6. Cleanup
    sem_destroy(&update_sem);
    close(can_socket_fd);
    lgGpiochipClose(gpio_h);
    
    return 0;
}