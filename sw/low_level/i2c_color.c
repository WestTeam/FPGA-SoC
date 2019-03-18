
#include <string.h>
#include <stdint.h>
#include <limits.h>

#include <tools.h>
#include <i2c.h>


//the I2C address for the color sensor 
#define COLOR_SENSOR_ADDR  0x39 
#define REG_CTL 0x80
#define REG_TIMING 0x81
#define REG_INT 0x82
#define REG_INT_SOURCE 0x83
#define REG_ID 0x84
#define REG_GAIN 0x87
#define REG_LOW_THRESH_LOW_BYTE 0x88
#define REG_LOW_THRESH_HIGH_BYTE 0x89
#define REG_HIGH_THRESH_LOW_BYTE 0x8A
#define REG_HIGH_THRESH_HIGH_BYTE 0x8B
//The REG_BLOCK_READ and REG_GREEN_LOW direction are the same
#define REG_BLOCK_READ 0xD0 
#define REG_GREEN_LOW 0xD0
#define REG_GREEN_HIGH 0xD1
#define REG_RED_LOW 0xD2
#define REG_RED_HIGH 0xD3
#define REG_BLUE_LOW 0xD4
#define REG_BLUE_HIGH 0xD5
#define REG_CLEAR_LOW 0xD6
#define REG_CLEAR_HIGH 0xD7
#define CTL_DAT_INIITIATE 0x03
#define CLR_INT 0xE0

//Timing Register
#define SYNC_EDGE 0x40
#define INTEG_MODE_FREE 0x00
#define INTEG_MODE_MANUAL 0x10
#define INTEG_MODE_SYN_SINGLE 0x20
#define INTEG_MODE_SYN_MULTI 0x30
 
#define INTEG_PARAM_PULSE_COUNT1 0x00
#define INTEG_PARAM_PULSE_COUNT2 0x01
#define INTEG_PARAM_PULSE_COUNT4 0x02
#define INTEG_PARAM_PULSE_COUNT8 0x03

//Interrupt Control Register 
#define INTR_STOP 40
#define INTR_DISABLE 0x00
#define INTR_LEVEL 0x10
#define INTR_PERSIST_EVERY 0x00
#define INTR_PERSIST_SINGLE 0x01

//Interrupt Souce Register
#define INT_SOURCE_GREEN 0x00
#define INT_SOURCE_RED 0x01
#define INT_SOURCE_BLUE 0x10
#define INT_SOURCE_CLEAR 0x03

//Gain Register
#define GAIN_1 0x00
#define GAIN_4 0x10
#define GAIN_16 0x20
#define GANI_64 0x30
#define PRESCALER_1 0x00
#define PRESCALER_2 0x01
#define PRESCALER_4 0x02
#define PRESCALER_8 0x03
#define PRESCALER_16 0x04
#define PRESCALER_32 0x05
#define PRESCALER_64 0x06



uint8_t i2c_color_get(volatile uint32_t *base,uint16_t *R,uint16_t *G, uint16_t *B, uint16_t *C)
{
    uint8_t i;
    uint8_t buff[8];
    uint8_t error = 0;
 
    for (i=0;i<8;i++)
    {
        uint8_t addr = 0x90+i;

        error += i2c_master_tx_rx(base,COLOR_SENSOR_ADDR,&addr,1,&buff[i],1);
    }
    
    *G = (uint16_t)buff[1] << 8 | buff[0];
    *R = (uint16_t)buff[3] << 8 | buff[2];
    *B = (uint16_t)buff[5] << 8 | buff[4];
    *C = (uint16_t)buff[7] << 8 | buff[6];

    return error;
}


uint8_t i2c_color_init(volatile uint32_t *base)
{
    uint8_t error=0;

	//GroveColorSensor::setTimingReg(); 


    uint8_t buf[2];

    buf[0] = REG_TIMING;
    buf[1] = INTEG_MODE_FREE | INTEG_PARAM_PULSE_COUNT1;
    error += i2c_master_tx(base,COLOR_SENSOR_ADDR,buf,2);


	//GroveColorSensor::setInterruptSourceReg();  


    buf[0] = REG_INT_SOURCE;
    buf[1] = INT_SOURCE_CLEAR;
    error += i2c_master_tx(base,COLOR_SENSOR_ADDR,buf,2);

	//GroveColorSensor::setInterruptControlReg();

    buf[0] = REG_INT;
    buf[1] = INTR_LEVEL | INTR_PERSIST_EVERY;
    error += i2c_master_tx(base,COLOR_SENSOR_ADDR,buf,2);



	//GroveColorSensor::setGain(); 
    buf[0] = REG_GAIN;
    buf[1] = GAIN_4 | PRESCALER_1;
    error += i2c_master_tx(base,COLOR_SENSOR_ADDR,buf,2);


	//GroveColorSensor::setEnableADC();

    buf[0] = REG_CTL;
    buf[1] = CTL_DAT_INIITIATE;
    error += i2c_master_tx(base,COLOR_SENSOR_ADDR,buf,2);


    uint16_t test;
    error += i2c_color_get(base,&test,&test,&test,test);

    return error;

}



