
//#include <string.h>
//#include <stdint.h>
//#include <limits.h>

#include <tools.h>
//#include <i2c.h>
//#include <i2c_color.h>
//#include <i2c_proximity.h>

#include <servo_ctrl.h>

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

typedef struct {
    uint8_t enabled;
    uint8_t bus_id;
    uint8_t protocol;
    //uint32_t retry_count;
    uint16_t target;
    //uint16_t speed;
    uint16_t current_pos;
    uint16_t current_load;
    uint8_t voltage;
    uint8_t temp;
    uint32_t ts_last_op;
    uint32_t ts_wait_period; // if 0, disabled 
    uint8_t error_count;
    uint8_t error;
} servo_device_t;

#define DYNAMIXEL_WAIT_DEFAULT_MS (1)
#define DYNAMIXEL_WAIT_SET_POS_MS (10)


#define SERVO_COUNT 8

typedef struct
{
    uint32_t timer;
    // latest cmd received
    uint8_t cmd_valid;
    uint8_t cmd_id;
    uint8_t moving;
    uint8_t instr_reg;
    servo_device_t servos[SERVO_COUNT];

} low_level_data_t;




void low_level_update_cmd(volatile low_level_mapping_t* regs, low_level_data_t* data)
{
    uint8_t cmd_id = regs->cmd_id;

    if (regs->cmd_valid == 1 && (cmd_id != data->cmd_id || data->cmd_valid == 0))
    {
        //print_int(cmd_id,1);
        uint8_t cmd_type = regs->cmd_type;
        uint8_t cmd_instr = regs->cmd.raw.instr;
        uint8_t error = 0;
        uint16_t read_value = 0;

        servo_device_t *servo = &data->servos[regs->cmd_dev_id];

        // we check again the pos_id is the same as well as valid, to make sure the data we read is ok.
        if (regs->cmd_id == cmd_id && regs->cmd_valid == 1)
        {
            //uint8_t protocol = 0;
            //uint8_t on_hold = 0;

#define SERVO_CMD_TYPE_RAW 0
#define SERVO_CMD_TYPE_REGISTER_DEV 1
#define SERVO_CMD_TYPE_SET_POS 2
#define SERVO_CMD_TYPE_SET_POS_AND_SPEED 3
#define SERVO_CMD_TYPE_SET_ACTION 4
#define SERVO_CMD_TYPE_GET_STATUS 5
#define SERVO_CMD_TYPE_SET_ENABLE 6


            // depending on the command type, we save the data:
            switch (cmd_type) 
            {
#define RAW_WR8B 0
#define RAW_WR16B 1
#define RAW_RD8B 2
#define RAW_RD16B 3
#define RAW_ACTION 4

                case SERVO_CMD_TYPE_RAW:
                    switch (cmd_instr)
                    {
                        case RAW_WR8B:
                            error = servo_ctrl_set_8b_retry(regs->cmd.raw.protocol,regs->cmd.raw.on_hold,regs->cmd_dev_id,regs->cmd.raw.addr,(uint8_t)(regs->cmd.raw.data & 0xff));
                            break;
                        case RAW_WR16B:
                            error = servo_ctrl_set_16b_retry(regs->cmd.raw.protocol,regs->cmd.raw.on_hold,regs->cmd_dev_id,regs->cmd.raw.addr,regs->cmd.raw.data);

                            /*jtaguart_puts("write\n");
                            print_int(regs->cmd.raw.protocol,1);
                            print_int(regs->cmd.raw.on_hold,1);
                            print_int(regs->cmd_dev_id,1);
                            print_int(regs->cmd.raw.addr,1);
                            print_int(regs->cmd.raw.data,1);*/
                            break;
                        case RAW_RD8B:
                            error = servo_ctrl_get_8b_retry(regs->cmd.raw.protocol,regs->cmd_dev_id,regs->cmd.raw.addr,(uint8_t*)&read_value);
                            regs->cmd.raw.data = read_value;
                            break;
                        case RAW_RD16B:
                            error = servo_ctrl_get_16b_retry(regs->cmd.raw.protocol,regs->cmd_dev_id,regs->cmd.raw.addr,&read_value);
                            regs->cmd.raw.data = read_value;
                            //jtaguart_puts("read\n");
                            break;
                        case RAW_ACTION:
                            error = servo_ctrl_set_action(regs->cmd.raw.protocol);
                            break;

                    }
                    break;

                case SERVO_CMD_TYPE_REGISTER_DEV:
                    servo->error_count = 0;
                    servo->bus_id = regs->cmd.register_dev.bus_id;
                    servo->protocol = regs->cmd.register_dev.protocol;
                    servo->enabled = 0;
                    data->moving &= ~(1 << regs->cmd_dev_id);
                    data->instr_reg &= ~(1 << regs->cmd_dev_id);
                    servo->ts_wait_period = 0;

                    // we disable the servo 
                    error = servo_ctrl_set_enable(servo->protocol,0,servo->bus_id,0);

                    uint8_t i;
                    for (i=0;i<SERVO_COUNT;i++)
                    {
                        if (data->servos[i].bus_id == regs->cmd.register_dev.bus_id && data->servos[i].protocol == servo->protocol)
                            data->servos[i].enabled = 0;
                    }


                    ts_start(&servo->ts_last_op);
                    servo->ts_wait_period = MS_TO_CYCLES(DYNAMIXEL_WAIT_DEFAULT_MS);

                    //jtaguart_puts("reg dev\n");
                    /*print_int(servo->protocol,1);
                    print_int(servo->bus_id,1);*/
                    break;

                case SERVO_CMD_TYPE_SET_POS:
                case SERVO_CMD_TYPE_SET_POS_AND_SPEED:
                    error = 0;
                    if (servo->enabled == 0)
                        break;


                    if (servo->ts_wait_period)
                    {
                        ts_wait_until_elapsed(servo->ts_last_op,servo->ts_wait_period);
                    }

                    if (cmd_type == SERVO_CMD_TYPE_SET_POS_AND_SPEED)
                        error = servo_ctrl_set_speed(servo->protocol,0,servo->bus_id,regs->cmd.set_pos_and_speed.speed);
                    
                    if (!error)
                        error = servo_ctrl_set_pos(servo->protocol,regs->cmd.set_pos.on_hold,servo->bus_id,regs->cmd.set_pos.pos);

                    if (error == 0)
                    {
                        data->timer = 0;
                        /*jtaguart_puts("set pos\n");
                        print_int(servo->protocol,1);
                        print_int(regs->cmd.set_pos.on_hold,1);
                        print_int(servo->bus_id,1);
                        print_int(regs->cmd.set_pos.pos,1);*/

                        servo->target = regs->cmd.set_pos.pos;
                        
                        data->moving |= (1 << regs->cmd_dev_id);
                        regs->cmd_valid = data->moving;

                        if (regs->cmd.set_pos.on_hold == 1)
                            data->instr_reg |= (1 << regs->cmd_dev_id);

                        if (regs->cmd.set_pos.wait_position == 1 && regs->cmd.set_pos.on_hold == 0)
                        {

                            delay(MS_TO_CYCLES(DYNAMIXEL_WAIT_SET_POS_MS));

                            uint8_t i;
                            #define CHECK_COUNT 10
                            #define ANGLE_ERROR 20
                            for (i=0;i<CHECK_COUNT;i++)
                            {
                                uint16_t current_pos;
                                uint8_t get_error;
                                get_error = servo_ctrl_get_pos(servo->protocol,servo->bus_id,&current_pos);
                                if (get_error == 0)
                                {
                                    int16_t diff = (int16_t)current_pos-(int16_t)regs->cmd.set_pos.pos;
                                    /*print_int(diff,1);
                                    print_int(current_pos,1);
                                    print_int(regs->cmd.set_pos.pos,1);*/

                                    if (diff > -ANGLE_ERROR && diff < ANGLE_ERROR)
                                        break;
                                }
                                delay(MS_TO_CYCLES(20));
                            }
                            if (i == CHECK_COUNT)
                                error = SERVO_CTRL_ERROR_TIMEOUT;
                        }
                    } else {
                        //jtaguart_puts("error set pos ");
                        //print_int(error,1);
                        //print_int(servo_ctrl_get_last_error(),1);
                        data->moving &= ~(1 << regs->cmd_dev_id);
                        regs->cmd_valid = data->moving;
                    }
                    

                    ts_start(&servo->ts_last_op);
                    servo->ts_wait_period = MS_TO_CYCLES(DYNAMIXEL_WAIT_SET_POS_MS);
    
                    if (error)
                        servo->error_count++;
                    break;

                case SERVO_CMD_TYPE_SET_ACTION:

                    data->instr_reg = 0;
                    //data->timer = 0;
                    servo_ctrl_set_action(SERVO_CTRL_FEETECH);
                    servo_ctrl_set_action(SERVO_CTRL_DYNAMIXEL);
                    servo_ctrl_set_action(SERVO_CTRL_FEETECH);
                    servo_ctrl_set_action(SERVO_CTRL_DYNAMIXEL);
                    servo_ctrl_set_action(SERVO_CTRL_FEETECH);
                    servo_ctrl_set_action(SERVO_CTRL_DYNAMIXEL);


                    if (regs->cmd.set_pos.wait_position == 1)
                    {
                        

                    }
                    break;
                case SERVO_CMD_TYPE_GET_STATUS:
                {

                    if (servo->ts_wait_period)
                    {
                        ts_wait_until_elapsed(servo->ts_last_op,servo->ts_wait_period);
                    }

                    error = servo_ctrl_get_status(servo->protocol, servo->bus_id, &servo->current_pos, &servo->current_load, &servo->voltage, &servo->temp);
                    regs->cmd.get_status.position = servo->current_pos;
                    regs->cmd.get_status.load = servo->current_load;
                    regs->cmd.get_status.voltage = servo->voltage;
                    regs->cmd.get_status.temp = servo->temp;
                    /*
                    print_int(position,1);
                    print_int(load,1);
                    print_int(voltage,1);
                    print_int(temp,1);
                    print_int(error,1);*/

                    ts_start(&servo->ts_last_op);
                    servo->ts_wait_period = MS_TO_CYCLES(DYNAMIXEL_WAIT_DEFAULT_MS);

                    if (error)
                        servo->error_count++;
                    break;
                }
                case SERVO_CMD_TYPE_SET_ENABLE:

                    if (servo->ts_wait_period)
                    {
                        ts_wait_until_elapsed(servo->ts_last_op,servo->ts_wait_period);
                    }

                    error = servo_ctrl_set_enable(servo->protocol, regs->cmd.set_enable.on_hold, servo->bus_id, regs->cmd.set_enable.enable);
                    if (error == 0)
                        servo->enabled = regs->cmd.set_enable.enable;

                    ts_start(&servo->ts_last_op);
                    servo->ts_wait_period = MS_TO_CYCLES(DYNAMIXEL_WAIT_DEFAULT_MS);

                    if (error)
                        servo->error_count++;
                    break;

            }



            //regs->cmd_out_data_read = read_value;
            regs->cmd_out_error = error;


            data->cmd_valid = 1;
            data->cmd_id = cmd_id;
            regs->cmd_out_ack = cmd_id+1;
            //print_int(cmd_id+1,1);
            //print_int(error,1);
        }
    }

}



int main()
{
    volatile low_level_mapping_t* regs = (low_level_mapping_t*)pio_n;
    low_level_data_t low_level;

    jtaguart_puts("LL Init\n");

    memset((void*)&low_level,0,sizeof(low_level_data_t));
    memset((void*)regs,0,sizeof(*regs));

    uint8_t uart_conf = 50000000/1000000;

    uart_rs232_configure(uart_conf);
    //uart_rs232_configure(48);


    uint16_t i;
  
    char chr;

    for (;;)
    {
        low_level_update_cmd(regs,&low_level);


        if (ts_is_elapsed(low_level.timer,MS_TO_CYCLES(50)))
        {
            ts_start(&low_level.timer);

            uint8_t bits_moving = low_level.moving;
            for (i=0;i<SERVO_COUNT;i++)
            {

                if (low_level.servos[i].ts_wait_period && ts_is_elapsed(low_level.servos[i].ts_last_op,low_level.servos[i].ts_wait_period))
                {
                    low_level.servos[i].ts_wait_period = 0;
                }

                if (low_level.servos[i].enabled && low_level.servos[i].ts_wait_period == 0 && ((low_level.moving>>i)&0x01) && !((low_level.instr_reg>>i)&0x01))
                {
                    uint8_t moving = 0;
                    {
                        uint16_t current_pos;
                        uint8_t get_error;
                        get_error = servo_ctrl_get_pos(low_level.servos[i].protocol,low_level.servos[i].bus_id,&current_pos);

                        ts_start(&low_level.servos[i].ts_last_op);
                        low_level.servos[i].ts_wait_period = MS_TO_CYCLES(DYNAMIXEL_WAIT_DEFAULT_MS);

                        if (get_error == 0)
                        {
                            int16_t diff = (int16_t)current_pos-(int16_t)low_level.servos[i].target;

                            low_level.servos[i].current_pos = current_pos;

                            if (!(diff > -ANGLE_ERROR && diff < ANGLE_ERROR))
                            {
                                moving = 1;
                            }
                        }


                        if (get_error)
                            low_level.servos[i].error_count++;
                    }
                    if (!moving)
                        bits_moving &= ~(1 << i);
                }
            }
            low_level.moving = bits_moving;
            regs->cmd_valid = bits_moving;
        }



        chr = jtaguart_getchar();
        if (chr != 0)
        {
            switch (chr)
            {

                case 'e':
                    print_int(servo_ctrl_error_counter,1);
                    print_int(servo_ctrl_error_missing_response,1);
                    print_int(servo_ctrl_error_checksum_error,1);
                    print_int(servo_ctrl_get_last_error(),1);
                    print_int(servo_ctrl_get_last_error_raw(),1);
                    break;

                case 'd':
                    for (i=0;i<SERVO_COUNT;i++)
                    {
                        if (low_level.servos[i].enabled == 1) {
                            jtaguart_puts("servo dev ");
                            print_int(i,0);
                            jtaguart_puts(" bus ");
                            print_int(low_level.servos[i].bus_id,1);
                            print_int(((low_level.moving>>i)&0x01),1);
                            print_int(((low_level.instr_reg>>i)&0x01),1);
                            print_int(low_level.servos[i].target,1);
                            print_int(low_level.servos[i].current_pos,1);
                            print_int(low_level.servos[i].error_count,1);

                        }
                    }
                    break;        

                case 'u':
                    uart_rs232_configure(--uart_conf);
                    print_int(uart_conf,1);
                    break;   
                case 'U':
                    uart_rs232_configure(++uart_conf);
                    print_int(uart_conf,1);
                    break;   

            }
        }

    }

}
