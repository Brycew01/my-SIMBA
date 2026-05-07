// #include <pthread.h>
// #include <unistd.h>


#include "bmi270_port.h"




volatile uint32_t *iic_base;                // actual definition of the global pointer to the mmap'd AXI IIC base address
extern const uint8_t bmi270_config_file[];  //  forward declaration (telling the compiler that this array exists somewhere)


// takes a register offset and value and writes it to the AXI IIC controller
// steps byte by byte to the correct register, then casts to volatile uint32_t* to do a 32-bit write as required by the AXI bus.
void iic_write(uint32_t offset, uint32_t val) {
    *((volatile uint32_t*)((uint8_t*)iic_base + offset)) = val;
}


// same as iic_write but it returns the 32-bit register value instead of writing.
uint32_t iic_read(uint32_t offset) {
    return *((volatile uint32_t*)((uint8_t*)iic_base + offset));
}


// delay function
void bmi270_delay_us(uint32_t period, void *intf_ptr){


    (void)intf_ptr;
    struct timespec ts = {0, period * 1000};
    nanosleep(&ts, NULL);
}


// Resets and enables the controller once at startup before any transactions begin
void axi_iic_init(void) {
    iic_write(REG_SOFTR, 0xA);      // reset the AXI IIC controller
    bmi270_delay_us(50000, NULL);
    iic_write(REG_CR, 0x1);         // writes 1 to enable the control register
    bmi270_delay_us(10000, NULL);
}


void bmi270_read_reg(uint8_t reg, uint8_t addr, uint8_t *data, uint32_t data_len) {
   
    // START + address + write
    iic_write(REG_TX_FIFO, 0x100 | (addr << 1));    
    bmi270_delay_us(100, NULL);
   
    // Register address
    iic_write(REG_TX_FIFO, reg);
    bmi270_delay_us(100, NULL);
   
    // Repeated START + address + read
    iic_write(REG_TX_FIFO, 0x100 | (addr << 1) | 1);
    bmi270_delay_us(100, NULL);
   
    // STOP + read data_len bytes
    iic_write(REG_TX_FIFO, 0x200 | data_len);
    bmi270_delay_us(100, NULL);


    // loop through and write data to data array
    for (uint32_t i = 0; i < data_len; i++){
        uint32_t timeout = 1000;
        while ((iic_read(REG_SR) & 0x40) && timeout--);
        data[i] = iic_read(REG_RX_FIFO) & 0xFF;
    }
}


void bmi270_write_reg(uint8_t addr, uint8_t reg, const uint8_t* data, uint32_t data_len) {
   
    // START + address + write
    iic_write(REG_TX_FIFO, 0x100 | (addr << 1));
    bmi270_delay_us(100, NULL);


    // Register address
    iic_write(REG_TX_FIFO, reg);
    bmi270_delay_us(100, NULL);


    // loop through and send data
    for (uint32_t i = 0; i < data_len; i++){
        if ((data_len - 1) == i){
            // STOP
            iic_write(REG_TX_FIFO, 0x200 | data[i]);
            bmi270_delay_us(500, NULL);
        }
        else{
            // not the last, write data
            iic_write(REG_TX_FIFO, data[i]);
            bmi270_delay_us(100, NULL);
        }
    }
}


int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr){

    bmi270_i2c_ctx_t *ctx = (bmi270_i2c_ctx_t*)intf_ptr;
    uint8_t dev_addr = ctx->addr;
    bmi270_read_reg(reg_addr, dev_addr, data, len);
    return 0;   // BMI2_OK
}


int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr){

    bmi270_i2c_ctx_t *ctx = (bmi270_i2c_ctx_t*)intf_ptr;
    uint8_t dev_addr = ctx->addr;
    bmi270_write_reg(dev_addr, reg_addr, data, len);
    return 0;   // BMI2_OK
}


int8_t bmi270_kria_init(bmi270_i2c_ctx_t *ctx, struct bmi2_dev *dev) {
    if (!ctx || !dev) return BMI2_E_INVALID_INPUT;


    dev->intf = BMI2_I2C_INTF;
    dev->read = bmi2_i2c_read;
    dev->write = bmi2_i2c_write;
    dev->delay_us = bmi270_delay_us;
    dev->intf_ptr = ctx;
    dev->read_write_len = 32;
    dev->config_file_ptr = bmi270_config_file;
    dev->config_size = 8192;


    printf("Starting bmi270_init...\n");
    fflush(stdout);
    int8_t rslt = bmi270_init(dev);
    if (rslt != BMI2_OK) return rslt;


    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sens_list, 2, dev);
    if (rslt != BMI2_OK) return rslt;


    struct bmi2_sens_config cfg[2];
    cfg[0].type = BMI2_ACCEL;
    cfg[1].type = BMI2_GYRO;

    rslt = bmi2_get_sensor_config(cfg, 2, dev);
    if (rslt != BMI2_OK) return rslt;


    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    cfg[1].cfg.gyr.filter_perf = BMI2_GYR_OSR4_MODE;

    rslt = bmi2_set_sensor_config(cfg, 2, dev);
    if (rslt != BMI2_OK) return rslt;

    bmi270_delay_us(20000, NULL);

    return BMI2_OK;
}


int8_t bmi270_kria_read_ag(struct bmi2_dev *dev, struct bmi2_sens_axes_data *acc, struct bmi2_sens_axes_data *gyr) {
    struct bmi2_sens_data sens = {0};

    int8_t rslt = bmi2_get_sensor_data(&sens, dev);
    if (rslt != BMI2_OK) return rslt;

    if (acc) *acc = sens.acc;
    if (gyr) *gyr = sens.gyr;

    return BMI2_OK;
}
