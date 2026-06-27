#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>//for read write functions
#include <fcntl.h>//for open , the open func opens the file for reading and writing it
#include <stdint.h>
#include <stdio.h>

#define CFG_AIN0  0xC083    // MUX=100 AIN0 vs GND
#define CFG_AIN1  0xD083    // MUX=101 AIN1 vs GND
#define CFG_AIN2  0xE083    // MUX=110 AIN2 vs GND
#define CFG_AIN3  0xF081    // MUX=111 AIN3 vs GND

#define RAW_AT_3V3  17589L

#define I2C_BUS          "/dev/i2c-1"
#define ADS1115_ADDR_1     0x48
#define ADS1115_ADDR_2     0x49
#define ADS1115_ADDR_3     0x4A //to be set
#define ADS1115_REG_CONV 0x00
#define ADS1115_REG_CFG  0x01

static int i2c_fd_1  = -1;
static int i2c_fd_2  = -1;
static int i2c_fd_3  = -1;
//=================================================================================================
static int ads1115_open(uint8_t addr, int* i2c_fd) {
    *i2c_fd = open(I2C_BUS, O_RDWR);
    if (*i2c_fd < 0) { perror("open /dev/i2c-1"); return -1; }
    if (ioctl(*i2c_fd, I2C_SLAVE, addr) < 0) {
        perror("ioctl I2C_SLAVE"); close(*i2c_fd); return -1;
    }
    return 0;
}

static void ads1115_close(int* i2c_fd) {
    if (*i2c_fd >= 0) { close(*i2c_fd); *i2c_fd = -1; }
}

static int ads1115_write_reg(uint8_t reg, uint16_t val,int* i2c_fd) {
    uint8_t buf[3] = {reg, (val >> 8) & 0xFF, val & 0xFF};
    if (write(*i2c_fd, buf, 3) != 3) { perror("I2C write"); return -1; }
    return 0;
}

// Single-shot conversion → returns 0–4095 (scaled to 3.3 V full scale)
static uint16_t ads1115_read_channel(uint16_t cfg,int* i2c_fd) {
    if (ads1115_write_reg(ADS1115_REG_CFG, cfg, i2c_fd) < 0) return 0;

    // Poll OS bit in config register until conversion is complete (~8 ms max)
    for (int i = 0; i < 20; i++) {
        usleep(1000);   // 1 ms per poll
        uint8_t ptr = ADS1115_REG_CFG;
        if (write(*i2c_fd, &ptr, 1) != 1) break;
        uint8_t b[2] = {0};
        if (read(*i2c_fd, b, 2) != 2) break;
        if (b[0] & 0x80) break;     // OS bit HIGH → done
    }

    // Read conversion register
    uint8_t ptr = ADS1115_REG_CONV;
    if (write(*i2c_fd, &ptr, 1) != 1) { perror("I2C write ptr"); return 0; }
    uint8_t b[2] = {0};
    if (read(*i2c_fd, b, 2) != 2)    { perror("I2C read");       return 0; }
    int16_t raw = (int16_t)((b[0] << 8) | b[1]);

    // Clamp: negative = below GND (noise), above RAW_AT_3V3 = over 3.3 V
    if (raw < 0)                        raw = 0;
    if (raw > (int16_t)RAW_AT_3V3)     raw = (int16_t)RAW_AT_3V3;

    // Scale raw 0..17589 → adc12 0..4095
    return (uint16_t)((raw * 4095L) / RAW_AT_3V3);
}


//===================================================================

void main(){

    while(1){

      if (ads1115_open(ADS1115_ADDR_1,&i2c_fd_1) < 0) {
        fprintf(stderr, "ADS1115 init failed — check I2C wiring and 'sudo raspi-config'\n");

        };


       //=======Read==========
       uint16_t adc_1[4];
       //uint16_t adc_2[4];
       //uint16_t adc_3[4];


       adc_1[0] = ads1115_read_channel(CFG_AIN0,&i2c_fd_1);
       adc_1[1] = ads1115_read_channel(CFG_AIN1,&i2c_fd_1);
       adc_1[2] = ads1115_read_channel(CFG_AIN2,&i2c_fd_1);
       adc_1[3] = ads1115_read_channel(CFG_AIN3,&i2c_fd_1);
       printf("%4u  %4u  %4u  %4u\n", adc_1[0], adc_1[1], adc_1[2], adc_1[3]);
       ads1115_close(&i2c_fd_1);

if (ads1115_open(ADS1115_ADDR_2,&i2c_fd_2) < 0) {
        fprintf(stderr, "ADS1115 init failed — check I2C wiring and 'sudo raspi-config'\n");

        };


       //=======Read==========
       uint16_t adc_2[4];
       //uint16_t adc_2[4];
       //uint16_t adc_3[4];


       adc_2[0] = ads1115_read_channel(CFG_AIN0,&i2c_fd_2);
       adc_2[1] = ads1115_read_channel(CFG_AIN1,&i2c_fd_2);
       adc_2[2] = ads1115_read_channel(CFG_AIN2,&i2c_fd_2);
       adc_2[3] = ads1115_read_channel(CFG_AIN3,&i2c_fd_2);
      printf("%4u  %4u  %4u  %4u\n", adc_2[0], adc_2[1], adc_2[2], adc_2[3]);
       ads1115_close(&i2c_fd_2);

if (ads1115_open(ADS1115_ADDR_3,&i2c_fd_3) < 0) {
        fprintf(stderr, "ADS1115 init failed — check I2C wiring and 'sudo raspi-config'\n");

        };


       //=======Read==========
       uint16_t adc_3[4];
       //uint16_t adc_2[4];
       //uint16_t adc_3[4];


       adc_3[0] = ads1115_read_channel(CFG_AIN0,&i2c_fd_3);
       adc_3[1] = ads1115_read_channel(CFG_AIN1,&i2c_fd_3);
       adc_3[2] = ads1115_read_channel(CFG_AIN2,&i2c_fd_3);
       adc_3[3] = ads1115_read_channel(CFG_AIN3,&i2c_fd_3);
       printf("%4u  %4u  %4u  %4u\n", adc_3[0], adc_3[1], adc_3[2], adc_3[3]);
       ads1115_close(&i2c_fd_3);


}
}



#WORKING
