
#include <stdint.h>


uint8_t i2c_proximity_init(volatile uint32_t *base);
uint8_t i2c_proximity_get(volatile uint32_t *base,uint8_t *dist);
