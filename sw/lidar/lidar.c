
//#include <string.h>
//#include <stdint.h>
//#include <limits.h>

#include <tools.h>



volatile uint32_t* pio_n = (volatile uint32_t*)(0x01000000);

typedef struct
{
    uint8_t reset;
    uint8_t pgm_id;
    uint8_t arg[2];

    // ADC 1278 Muxed
    uint8_t  adc_drdy;  // in // 1
    uint8_t  adc_reset; // out
    uint8_t  adc_valid; // 
    uint8_t  adc_id;
    uint32_t adc_value; // 2

    // Color Sensor
    uint8_t  color_valid; // 3 = 50
    uint8_t  reserved2[3];
    uint16_t R; //4
    uint16_t G;
    uint16_t B; //5
    uint16_t C;
    // Distance sensors
    uint8_t  dist_valid; // 6
    uint8_t  dist_id;
    uint8_t  reserved3[2];
    uint32_t dist_value; // 7  

    // offset = 
    uint8_t cmd_valid; // IN 8
    uint8_t cmd_id; // IN
    uint8_t cmd_type; // IN
    uint8_t cmd_dev_id; // IN // local id from 0 to 7, except for raw command which is real bus id
    union {
        struct raw
        {
            uint8_t on_hold; // IN // 0 = write, 1 = reg write
            uint8_t protocol; // IN // 0 = FEETECH, 1 = DYNAMIXEL 
            uint8_t instr; // IN // WR8B,WR16B,RD8B,RD16B, ACTION
            uint8_t addr; // IN
            uint16_t data; // IN & OUT
        } raw;
        struct register_dev
        {
            uint8_t protocol; // IN // 0 = FEETECH, 1 = DYNAMIXEL 
            uint8_t bus_id; // read bus id
        } register_dev;
        struct set_pos
        {
            uint8_t on_hold;
            uint8_t wait_position; // if true, wait for the position to be updated
            uint16_t pos; // IN
        } set_pos;
        struct set_pos_and_speed
        {
            uint8_t on_hold;
            uint8_t wait_position; // if true, wait for the position to be updated
            uint16_t pos; // IN
            uint16_t speed; // IN
        } set_pos_and_speed;
        struct set_action
        {
            uint8_t wait_position; // if true, wait for the position to be updated
        } set_action;
        struct set_enable
        {
            uint8_t on_hold;
            uint8_t enable;
        } set_enable;
        struct get_status
        {
            uint16_t position; // OUT
            uint16_t load; // OUT
            uint8_t voltage; // OUT
            uint8_t temp; // OUT
        } get_status;
    } cmd;
    uint8_t cmd_out_ack; // OUT 11
    //uint16_t cmd_out_data_read; // OUT
    uint8_t cmd_out_error; // OUT

 
} __attribute__((packed)) low_level_mapping_t;




int main()
{
    volatile low_level_mapping_t* regs = (low_level_mapping_t*)pio_n;


    jtaguart_puts("Lidar Init\n");
}
