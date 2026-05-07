#pragma once


#include <errno.h>
#include "bmi2.h"
#include "bmi270.h"
#include <time.h>


// defines
#define AXI_IIC_BASE 0x800e0000
#define MAP_SIZE     0x10000

#define REG_SOFTR    0x40
#define REG_CR       0x100
#define REG_SR       0x104
#define REG_TX_FIFO  0x108
#define REG_RX_FIFO  0x10C

extern volatile uint32_t *iic_base;

typedef struct {
    uint8_t port;
    uint8_t addr;
} bmi270_i2c_ctx_t;


int8_t bmi270_kria_init(bmi270_i2c_ctx_t *ctx, struct bmi2_dev *dev);
int8_t bmi270_kria_read_ag(struct bmi2_dev *dev, struct bmi2_sens_axes_data *acc, struct bmi2_sens_axes_data *gyr);

void bmi270_read_reg(uint8_t reg, uint8_t addr, uint8_t *data, uint32_t data_len);
void bmi270_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, uint32_t data_len);
void bmi270_delay_us(uint32_t period, void* intf_ptr);
void axi_iic_init(void);
