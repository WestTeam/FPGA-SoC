


#include <stdint.h>

#define i2c_master_0 ((volatile uint32_t*)(0x01000440))
#define i2c_master_1 ((volatile uint32_t*)(0x01000480))


#define I2C_RESTART 0x1
#define I2C_NO_RESTART 0x0

#define I2C_STOP 0x1
#define I2C_NO_STOP 0x0

#define I2C_WRITE 0x0
#define I2C_READ 0x1




void i2c_master_configure(volatile uint32_t *base);
void i2c_master_enable(volatile uint32_t *base, uint8_t enable);

uint8_t i2c_master_tx(volatile uint32_t *base,
                               uint8_t   addr,
                               uint8_t  *buffer,
                               uint32_t  size);


uint8_t i2c_master_tx_rx(volatile uint32_t *base,
                                             uint8_t   addr,
                                             uint8_t  *txbuffer,
                                             uint32_t  txsize,
                                             uint8_t  *rxbuffer,
                                             uint32_t  rxsize);
