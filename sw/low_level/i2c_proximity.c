#include <string.h>
#include <stdint.h>
#include <limits.h>

#include <tools.h>
#include <i2c.h>

#define SWITCH_ADDR 0x20
#define PROXIMITY_ADDR 0x29
#define PROXIMITY_ADDR_NEW 0x2a


uint8_t i2c_proximity_init(volatile uint32_t *base)
{
    uint8_t error=0;


    uint8_t buf[3];
    uint8_t rx_buf[2];


    buf[0] = 0x03;
    buf[1] = ~(0x84);
    error += i2c_master_tx(base,SWITCH_ADDR,buf,2);
/*
    buf[0] = 0x00;
    buf[1] = 0x00;
    error += i2c_master_tx_rx(base,PROXIMITY_ADDR,buf,2,rx_buf,1);
    print_int(rx_buf[0],1);

    buf[0] = 0x02;
    buf[1] = 0x12;
    buf[2] = PROXIMITY_ADDR_NEW;
    error += i2c_master_tx(base,PROXIMITY_ADDR,buf,3);
*/

    buf[0] = 0x00;
    buf[1] = 0x00;
    rx_buf[0] = 0x00;
    error += i2c_master_tx_rx(base,PROXIMITY_ADDR_NEW,buf,2,rx_buf,1);
    //print_int(rx_buf[0],1);


    return error;

}

uint8_t i2c_proximity_get(volatile uint32_t *base,uint8_t *dist)
{
    #define STATE_PROXIMITY_LAUNCH_MEASURE 0
    #define STATE_PROXIMITY_GET_MEASURE 1

    static uint8_t state = STATE_PROXIMITY_LAUNCH_MEASURE;
    static uint32_t ts = 0;
    static uint8_t dist_last = 0;    


    //uint8_t i;
    uint8_t buf[3];
    //uint8_t rx_buf[2];
    uint8_t error = 0;

    if (state == STATE_PROXIMITY_LAUNCH_MEASURE)
    {

        buf[0] = 0x00;
        buf[1] = 0x18;
        buf[2] = 0x01;

        error += i2c_master_tx(base,PROXIMITY_ADDR,buf,3);
        ts_start(&ts);
        *dist = dist_last;
        if (error == 0)
            state = STATE_PROXIMITY_GET_MEASURE;

    } else if (state == STATE_PROXIMITY_GET_MEASURE) {
        if (ts_is_elapsed(ts, 5000000))
        {

            buf[0] = 0x00;
            buf[1] = 0x62;

            error += i2c_master_tx_rx(base,PROXIMITY_ADDR,buf,2,dist,1);

            dist_last = *dist;
            state = STATE_PROXIMITY_LAUNCH_MEASURE;
            //print_int(*dist,1);
        }
    }
    

    //delay(5000000);

   
/*
    buf[0] = 0x00;
    buf[1] = 0x14;

    error += i2c_master_tx_rx(base,PROXIMITY_ADDR_NEW,buf,2,dist,1);

    print_int(*dist,1);
*/


    //print_int(*dist,1);

    return error;
}
