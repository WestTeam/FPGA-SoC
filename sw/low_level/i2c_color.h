
#include <string.h>
#include <stdint.h>
#include <limits.h>

//#include <tools.h>


uint8_t i2c_color_init(volatile uint32_t *base);

uint8_t i2c_color_get(volatile uint32_t *base,uint16_t *R,uint16_t *G, uint16_t *B, uint16_t *C);

