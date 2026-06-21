#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// --- USER DEFINED STRUCTS ---
struct Control_Word {
    unsigned int on : 1;
    unsigned int not_off2 : 1;
    unsigned int not_off3 : 1;
    unsigned int enable : 1;
    unsigned int not_hlg_reset : 1;
    unsigned int not_rfg_stop : 1;
    unsigned int setpoint_enable : 1;
    unsigned int reset : 1;
    unsigned int N : 1;
    unsigned int D : 1;
    unsigned int R : 1;
    unsigned int brake_SW : 1;
    unsigned int Hillhold : 1;
    unsigned int creep_disable : 1;
    unsigned int empty : 1;
    unsigned int toggle_bit : 1;
};

struct Status_Word {
    unsigned int power_up_ready : 1;
    unsigned int ready : 1;
    unsigned int operation_enabled : 1;
    unsigned int fault : 1;
    unsigned int no_off2_SF : 1;
    unsigned int no_off3_SH : 1;
    unsigned int power_on_inhibit : 1;
    unsigned int warning : 1;
    unsigned int N_signal_applied : 1;
    unsigned int D_signal_applied : 1;
    unsigned int R_signal_applied : 1;
    unsigned int creep_active : 1;
    unsigned int Hillhold_active : 1;
    unsigned int current_limit_active : 1;
    unsigned int empty : 1;
    unsigned int toggle_bit : 1;
};

// --- GLOBALS ---
bool interlocking = false;
bool not_ready_state = false;
bool ready_to_switch_on = false;
bool ready_for_operation = false;
bool opeation = false;
int falut_detected = 0;
int r2d = 0; 


#define RX_STATUS_ID 0x181 
#define TX_CONTROL_ID 0x201 
#define CAN_IFACE "can0"    

// status parsing
void Status_parsing(struct Status_Word *status, uint8_t byte0, uint8_t byte1) {
    status->power_up_ready = (byte0 >> 0) & 0x01;
    status->ready = (byte0 >> 1) & 0x01;
    status->operation_enabled = (byte0 >> 2) & 0x01;
    status->fault = (byte0 >> 3) & 0x01;
    status->no_off2_SF = (byte0 >> 4) & 0x01;
    status->no_off3_SH = (byte0 >> 5) & 0x01;
    status->power_on_inhibit = (byte0 >> 6) & 0x01;
    status->warning = (byte0 >> 7) & 0x01;
    status->N_signal_applied = (byte1 >> 0) & 0x01;
    status->D_signal_applied = (byte1 >> 1) & 0x01;
    status->R_signal_applied = (byte1 >> 2) & 0x01;
    status->creep_active = (byte1 >> 3) & 0x01;
    status->Hillhold_active = (byte1 >> 4) & 0x01;
    status->current_limit_active = (byte1 >> 5) & 0x01;
    status->empty = (byte1 >> 6) & 0x01;
    status->toggle_bit = (byte1 >> 7) & 0x01;
}

void Pack_Control_CAN(struct Control_Word *control, uint8_t *byte0, uint8_t *byte1) {
    *byte0 = (control->on << 0) | (control->not_off2 << 1) | (control->not_off3 << 2) |
             (control->enable << 3) | (control->not_hlg_reset << 4) | (control->not_rfg_stop << 5) |
             (control->setpoint_enable << 6) | (control->reset << 7);

    *byte1 = (control->N << 0) | (control->D << 1) | (control->R << 2) |
             (control->brake_SW << 3) | (control->Hillhold << 4) | (control->creep_disable << 5) |
             (control->empty << 6) | (control->toggle_bit << 7);
}

void set_control_word(struct Control_Word *control, struct Status_Word *status) {
    control->toggle_bit = !control->toggle_bit;

    if (falut_detected == 1) {
        control->reset = !control->reset;
        falut_detected = 0;
        control->D = 0;
        return;
    }
    if (status->fault == 1) {
        control->reset = !control->reset;
        falut_detected = 1;
        control->on = 0;
        control->not_off2 = 0;
        control->not_off3 = 0;
        control->enable = 0;
        control->D = 0;
        return;
    }
    if (status->power_on_inhibit == 1 && status->power_up_ready == 0 &&
        status->ready == 0 && status->operation_enabled == 0 && status->fault == 0) {
        control->not_off2 = 1;
        control->not_off3 = 1;
        control->on = 0;
        control->D = 0;
        interlocking = true;
        return;
    }
    if (status->power_on_inhibit == 0 && status->power_up_ready == 0 &&
        status->operation_enabled == 0 && status->ready == 0 &&
        status->fault == 0 && status->no_off2_SF == 1 && status->no_off3_SH == 1) {
        interlocking = false;
        not_ready_state = true;
        control->not_off2 = 1;
        control->not_off3 = 1;
        control->on = 0;
        control->D = 0;
        return;
    }
    if (status->power_on_inhibit == 0 && status->power_up_ready == 1 &&
        status->ready == 0 && status->operation_enabled == 0 && status->fault == 0 &&
        status->no_off2_SF == 1 && status->no_off3_SH == 1) {
        ready_to_switch_on = true;
        not_ready_state = false;
        control->not_off2 = 1;
        control->not_off3 = 1;
        if (r2d == 0) {
            control->on = 0;
            control->D = 0;
        } else {
            control->on = 1;
        }
        return;
    }
    if (status->power_on_inhibit == 0 && status->power_up_ready == 1 &&
        status->ready == 1 && status->operation_enabled == 0 && status->fault == 0 &&
        status->no_off2_SF == 1 && status->no_off3_SH == 1) {
        ready_to_switch_on = false;
        ready_for_operation = true;
        control->enable = 1;
        control->on = 1;
        control->not_off2 = 1;
        control->not_off3 = 1;
        control->D = 1;
        return;
    }
    if (status->power_on_inhibit == 0 && status->power_up_ready == 1 &&
        status->ready == 1 && status->operation_enabled == 1 && status->fault == 0 &&
        status->no_off2_SF == 1 && status->no_off3_SH == 1) {
        opeation = true;
        ready_for_operation = false;
        control->enable = 1;
        control->on = 1;
        control->not_off2 = 1;
        control->not_off3 = 1;
        control->D = 1;
    }
}

// can setup
int setup_socketcan(const char *ifname) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    // Open raw socket
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket error");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind error");
        close(s);
        return -1;
    }

    return s;
}


int main(void) {
    int s = setup_socketcan(CAN_IFACE);
    if (s < 0) {
        return 1;
    }

    struct can_frame frame;
    struct Status_Word inverter_status = {0};
    struct Control_Word vcu_control = {0};

    printf("Starting VCU Node on %s...\n", CAN_IFACE);

    while (1) {
        // 1. Read incoming CAN frame
        int nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            perror("CAN read error");
            continue;
        }

        // 2. Filter config
        if (frame.can_id == RX_STATUS_ID) {
            
            // 3. status bytes 0 and 1 
            Status_parsing(&inverter_status, frame.data[0], frame.data[1]);

            // 4. control according to status word
            set_control_word(&vcu_control, &inverter_status);

            // 5. pack control
            uint8_t tx_byte0 = 0, tx_byte1 = 0;
            Pack_Control_CAN(&vcu_control, &tx_byte0, &tx_byte1);

            // 6. Response Frame
            struct can_frame tx_frame;
            tx_frame.can_id = TX_CONTROL_ID;
            tx_frame.can_dlc = 8; 
            tx_frame.data[0] = tx_byte0;
            tx_frame.data[1] = tx_byte1;
            
            // Zero out the remaining bytes (unless you have a torque command mapped here)
            for(int i = 2; i < 8; i++) {
                tx_frame.data[i] = 0x00; 
            }

            // 7. Write Frame bus
            if (write(s, &tx_frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
                perror("CAN write error");
            } 
           
        }
    }

    close(s);
    return 0;
}