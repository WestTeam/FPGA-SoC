
//#include <string.h>
//#include <stdint.h>
//#include <limits.h>

#include <tools.h>



volatile uint32_t* pio_n = (volatile uint32_t*)(0x01000000);

typedef struct
{
    uint8_t reset;
    uint8_t reserved[3];

    uint8_t pos_valid; // IN  DATA
    uint8_t pos_id; // IN  DATA
    int16_t pos_teta; // IN  DATA
    int16_t pos_x; // IN  DATA
    int16_t pos_y; // IN  DATA

} __attribute__((packed)) lidar_mapping_t;

typedef struct
{
    uint32_t data_length:30;
    uint8_t send_mode:2;
    uint8_t data_type;
} __attribute__((packed)) rplidar_hdr_response;

typedef struct
{
    uint8_t start_flag:1;
    uint8_t n_start_flag:1;
    uint8_t quality:6;
    uint8_t check_bit:1;
    uint16_t angle_q6:15;
    uint16_t distance_q2:16;
} __attribute__((packed)) rplidar_data_response;


#define RPLIDAR_CMD_GET_DEVICE_INFO 0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALT_STATUS 0x52
#define RPLIDAR_CMD_GET_SAMPLE_RATE 0x59
#define RPLIDAR_CMD_GET_LIDAR_CONF 0x84
#define RPLIDAR_CMD_STOP 0x25
#define RPLIDAR_CMD_RESET 0x40

#define RPLIDAR_CMD_SCAN_WITH_ROBOTPOS 0x19
#define RPLIDAR_CMD_SCAN 0x20
#define RPLIDAR_CMD_FORCE_SCAN 0x21
#define RPLIDAR_CMD_SCAN_EXPRESS 0x82
#define RPLIDAR_CMD_GET_ACC_BOARD_FLAG 0xFF
#define RPLIDAR_CMD_SET_MOTOR_PWM 0xF0
// STOP = 25
// RESET = 40
// SCAN = 20
// FORCE SCAN = 21
// SCAN EXPRESS (LEGACY/EXTENDED) = 82 05 00 00 00 00 00 22
// RET
// SCAN NORMAL = A5 5A 05 00 00 40 81
// SCAN EXPRESS (LEGACY) = A5 5A 54 00 00 40 82
// SCAN EXPRESS (EXTENDED) = A5 5A 84 00 00 40 84




typedef enum 
{ 
    TX_PROTO_IDLE, 
    TX_PROTO_WAIT_COMMAND_TYPE,
    TX_PROTO_END_CMD, // wait end of frame (1 ms pause), because some optionnal data can arrive 
} tx_protocol_state_t;

typedef enum 
{ 
    RX_PROTO_IDLE, // wait start flag1
    RX_PROTO_WAIT_START_FLAG2,
    RX_PROTO_WAIT_END_RESP_DESCR, // wait until we got the full response descriptor
    RX_PROTO_WAIT_END_MULTI_NORMAL_WITH_ROBOTPOS
} rx_protocol_state_t;

typedef enum 
{ 
    STATE_IDLE, 
    STATE_SCAN_NORMAL,
    STATE_SCAN_EXPRESS,
    STATE_SCAN_NORMAL_WITH_ROBOTPOS
} state_t;



typedef struct
{
    tx_protocol_state_t   tx_protocol_state;
    uint32_t              tx_protocol_ts; // used to store latest event time

    rx_protocol_state_t   rx_protocol_state;
    uint32_t              rx_protocol_ts; // used to store latest event time
    rplidar_hdr_response  rx_hdr;
    uint8_t               rx_hdr_index;
    rplidar_data_response rx_data;
    uint8_t               rx_data_index;


    uint32_t              rx_timeout_count;
    uint32_t              tx_timeout_count;

    uint32_t              rx_byte_count;
    uint32_t              tx_byte_count;

    uint32_t              rx_packet_count;
    uint32_t              tx_packet_count;

    uint32_t              tx_reset_count;

    state_t state;

    uint8_t pos_valid;
    uint8_t pos_id;
    int16_t pos_teta;
    int16_t pos_x;
    int16_t pos_y;
} local_data_t;


void update_position(volatile lidar_mapping_t* regs, local_data_t *data)
{
    uint8_t pos_id = regs->pos_id;
    if (regs->pos_valid == 1 && (pos_id != data->pos_id || data->pos_valid == 0))
    {
        //print_int(pos_id,1);
        //print_int(data->pos_id,1);
        //print_int(regs->odo.pos_id,1);
        
        int16_t pos_teta = regs->pos_teta; 
        int16_t pos_x    = regs->pos_x; 
        int16_t pos_y    = regs->pos_y; 
        
        // we check again the pos_id is the same as well as valid, to make sure the data we read is ok.
        if (regs->pos_id == pos_id && regs->pos_valid == 1)
        {
            data->pos_x = pos_x;
            data->pos_y = pos_y;
            data->pos_teta = pos_teta;

            data->pos_valid = 1;
            data->pos_id = pos_id;
        }
    }
}


#define UART_LIDAR_SIDE 0
#define UART_SW_SIDE 1


// here we try to intercept/sniff message from sw library to lidar
// main purpose is to detect if we are in scanning mode, and 
// especially if we detect a "special scan" mode designed by us 
// where we need to integrate robot position (x,y,teta) 
// in each coordinates the laser sends out

void sw_to_lidar_process(local_data_t *data)
{
    uint8_t rx_data;
    uint8_t error;

    // we get data if any
    uart_rs232_select(UART_SW_SIDE);
    error = uart_rs232_rx(&rx_data,0);

    switch (data->tx_protocol_state)
    {
        case TX_PROTO_IDLE:
            if (error == 0)
            {
                if (rx_data == 0xA5)
                {
                    //jtaguart_puts("F\n");
                    data->tx_protocol_state = TX_PROTO_WAIT_COMMAND_TYPE;
                    ts_start(&data->tx_protocol_ts);
                }
            }
            break;
        case TX_PROTO_WAIT_COMMAND_TYPE:
            if (error == 0)
            {
                data->tx_packet_count++;

                //data->tx_protocol_state = TX_PROTO_END_CMD;

                if (((rx_data >> 7) & 0x01) == 1)
                {
                    data->tx_protocol_state = TX_PROTO_END_CMD;
                    ts_start(&data->tx_protocol_ts);
                    //jtaguart_puts("TX_PROTO_END_CMD\n");
                }
                else
                    data->tx_protocol_state = TX_PROTO_IDLE;



                switch (rx_data)
                {

                    case RPLIDAR_CMD_GET_DEVICE_INFO:
                        data->state = STATE_IDLE;
                        //jtaguart_puts("DEVICE_INFO\n");
                        break;
                    case RPLIDAR_CMD_GET_DEVICE_HEALT_STATUS:
                        data->state = STATE_IDLE;
                        //jtaguart_puts("DEVICE_HEALT_STATUS\n");
                        break;
                    case RPLIDAR_CMD_GET_SAMPLE_RATE:
                        data->state = STATE_IDLE;
                        //jtaguart_puts("SAMPLE_RATE\n");
                        break;
                    case RPLIDAR_CMD_GET_LIDAR_CONF:
                        data->state = STATE_IDLE;
                        //jtaguart_puts("LIDAR_CONF\n");
                        break;
                    case RPLIDAR_CMD_STOP:
                        data->state = STATE_IDLE;
                        //jtaguart_puts("STOP\n");
                        break;
                    case RPLIDAR_CMD_RESET:
                        data->tx_reset_count++;
                        data->rx_protocol_state = RX_PROTO_IDLE;
                        data->state = STATE_IDLE;
                        //jtaguart_puts("RESET\n");
                        break;
                    case RPLIDAR_CMD_SCAN_WITH_ROBOTPOS:
                        // change back data
                        rx_data = RPLIDAR_CMD_SCAN;
                        data->state = STATE_SCAN_NORMAL_WITH_ROBOTPOS;
                        //jtaguart_puts("SCAN ROBOTPOS\n");
                        break;

                    case RPLIDAR_CMD_SCAN:
                        data->state = STATE_SCAN_NORMAL;
                        //jtaguart_puts("SCAN\n");
                        break;

                    case RPLIDAR_CMD_FORCE_SCAN:
                        data->state = STATE_SCAN_NORMAL;
                        //jtaguart_puts("FORCE_SCAN\n");
                        break;
                    case RPLIDAR_CMD_SCAN_EXPRESS:
                        data->state = STATE_SCAN_EXPRESS;
                        //jtaguart_puts("SCAN_EXPRESS\n");
                        break;
                    case RPLIDAR_CMD_GET_ACC_BOARD_FLAG:

                        data->rx_protocol_state = RX_PROTO_IDLE;
                        data->state = STATE_IDLE;
                        //jtaguart_puts("GET_A_B_F\n");
                        break;            
                    case RPLIDAR_CMD_SET_MOTOR_PWM:
                        //jtaguart_puts("SET_MOTOR_PWM\n");
                        break;


                    default:
                        data->state = STATE_IDLE;
                        //jtaguart_puts("RPLIDAR_CMD_UNKNOWN");
                        //print_int(rx_data,1);
                        //if (rx_data == 0xA5)
                        //    data->tx_protocol_state = TX_PROTO_IDLE;
                        break;

                }

            } else {
                // check timeout on this state
                if (ts_is_elapsed(data->tx_protocol_ts,MS_TO_CYCLES(1)))
                {
                    data->tx_protocol_state = TX_PROTO_IDLE;
                    data->tx_timeout_count++;
                }
            }
            break;
        case TX_PROTO_END_CMD:
            if (error == 0)
            {
                ts_start(&data->tx_protocol_ts);
            } else {
                if (ts_is_elapsed(data->tx_protocol_ts,MS_TO_CYCLES(1)))
                {
                    data->tx_protocol_state = TX_PROTO_IDLE;
                    data->tx_timeout_count++;
                }
            }
            break;
    }    


    // anyway, if we had data received from SW, we need to send it (identical or modified by the state machine)
    if (error == 0)
    {
        data->tx_byte_count++;
        
        uart_rs232_select(UART_LIDAR_SIDE);
        uart_rs232_tx(rx_data);
        //print_int(rx_data,1);
    }

}



// here we try to intercept/sniff message from lidar to sw library
// main purpose is to inject x/y/teta inside normal scan mode data
// to do so, we need to update the length of the response header
// when we are in state==STATE_SCAN_NORMAL_WITH_ROBOTPOS and also
// add at the end of each data 6 bytes more (x/y/teta)

void lidar_to_sw_process(local_data_t *data)
{
    uint8_t rx_data;
    uint8_t error;

    // we get data if any
    uart_rs232_select(UART_LIDAR_SIDE);
    error = uart_rs232_rx(&rx_data,0);

    uint8_t drop = 0;

    switch (data->rx_protocol_state)
    {
/*
    RX_PROTO_IDLE, // wait start flag1
    RX_PROTO_WAIT_START_FLAG2,
    RX_PROTO_WAIT_END_RESP_DESCR, // wait until we got the full response descriptor
    RX_PROTO_WAIT_END_MULTI_NORMAL_WITH_ROBOTPOS
*/
        case RX_PROTO_IDLE:
            if (error == 0)
            {
                if (rx_data == 0xA5 && data->state == STATE_SCAN_NORMAL_WITH_ROBOTPOS)
                {
                    drop = 1; // we drop to be able to re generate it later
                    data->rx_protocol_state = RX_PROTO_WAIT_START_FLAG2;
                    ts_start(&data->rx_protocol_ts);
                }
            }
            break;

        case RX_PROTO_WAIT_START_FLAG2:
            if (error == 0)
            {
                drop = 1; // we drop to be able to re generate it later
                if (rx_data == 0x5A)
                {
                    data->rx_protocol_state = RX_PROTO_WAIT_END_RESP_DESCR;
                    data->rx_hdr_index = 0;
                    ts_start(&data->rx_protocol_ts);
                } else {
                    data->rx_protocol_state = RX_PROTO_IDLE;
                }
            } else {
                // check timeout on this state
                if (ts_is_elapsed(data->rx_protocol_ts,MS_TO_CYCLES(1)))
                {
                    data->rx_protocol_state = RX_PROTO_IDLE;
                    data->rx_timeout_count++;
                }
            }

            break;

        case RX_PROTO_WAIT_END_RESP_DESCR:
            if (error == 0)
            {
                drop = 1; // we drop to be able to re generate it later

                ((uint8_t*)&data->rx_hdr)[data->rx_hdr_index++] = rx_data;
                if (data->rx_hdr_index == sizeof(data->rx_hdr))
                {
                    data->rx_packet_count++;

                    // we check if we got the good type of message
                    if (data->rx_hdr.data_type == 0x81 && data->rx_hdr.data_length == 5 && data->rx_hdr.send_mode == 0x01)
                    {
                        // we then modify the datatype to custom 0x80 & length = (5+2+2+2)
                        data->rx_hdr.data_type = 0x80;
                        data->rx_hdr.data_length = 5+2+2+2;
                        data->rx_protocol_state = RX_PROTO_WAIT_END_MULTI_NORMAL_WITH_ROBOTPOS;
                        data->rx_data_index = 0;
                    } else {
                        data->rx_protocol_state = RX_PROTO_IDLE;
                    }

                    uart_rs232_select(UART_SW_SIDE);
                    uart_rs232_tx(0xA5);
                    uart_rs232_tx(0x5A);
                    uart_rs232_tx_frame((uint8_t*)&data->rx_hdr,sizeof(data->rx_hdr));

                }
            }
            break;


        case RX_PROTO_WAIT_END_MULTI_NORMAL_WITH_ROBOTPOS:
            if (error == 0)
            {
                //drop = 0;

                ((uint8_t*)&data->rx_data)[data->rx_data_index++] = rx_data;
                if (data->rx_data_index == sizeof(data->rx_data))
                {
                    // we check if we got the good type of message
                    if (data->rx_data.start_flag != data->rx_data.n_start_flag && data->rx_data.check_bit == 1)
                    {
                        if (data->state != STATE_SCAN_NORMAL_WITH_ROBOTPOS)
                            data->rx_protocol_state = RX_PROTO_IDLE;

                        uart_rs232_select(UART_SW_SIDE);
                        drop = 1; // just drop the current byte because we need to generate more
                        uart_rs232_tx(rx_data);
                        uart_rs232_tx_frame((uint8_t*)&data->pos_x,sizeof(data->pos_x));
                        uart_rs232_tx_frame((uint8_t*)&data->pos_y,sizeof(data->pos_y));
                        uart_rs232_tx_frame((uint8_t*)&data->pos_teta,sizeof(data->pos_teta));

                        data->rx_data_index = 0;
                    } else {
                        data->rx_protocol_state = RX_PROTO_IDLE;
                    }                    
                }
            }
            break;

    }


    // anyway, if we had data received from SW, we need to send it (identical or modified by the state machine)
    if (error == 0 && drop == 0)
    {
        data->rx_byte_count++;

        uart_rs232_select(UART_SW_SIDE);
        uart_rs232_tx(rx_data);
        //print_int(rx_data,1);
    }

}




int main()
{
    volatile lidar_mapping_t* regs = (lidar_mapping_t*)pio_n;
    local_data_t data;

    memset(&data,0,sizeof(data));

    jtaguart_puts("Lidar Init\n");

    uart_rs232_select(UART_LIDAR_SIDE);
    uart_rs232_configure(CPU_FREQ_HZ/115200);
    uart_rs232_select(UART_SW_SIDE);
    uart_rs232_configure(CPU_FREQ_HZ/(256000));

    uint8_t state = 0;
    uint8_t data_size = 0;
    uint8_t data_type = 0;

    rplidar_data_response data_response;
    uint8_t print = 0;
    uint32_t count = 0;
    while (1) 
    {

        update_position(regs,&data);

        uint8_t error;    
        uint8_t rx_data = 0x00;
    /*
        uart_rs232_select(1);
        error = uart_rs232_rx(&data,0);
        if (error == 0)
        {
            uart_rs232_select(0);
            uart_rs232_tx(data);
        }
*/
        sw_to_lidar_process(&data);
        lidar_to_sw_process(&data);


        char chr;
        chr = jtaguart_getchar();
        if (chr != 0)
        {
            jtaguart_puts("State:");
            print_int(data.state,1);
            jtaguart_puts("RX:\n");
            print_int(data.rx_protocol_state,1);
            print_int(data.rx_byte_count,1);
            print_int(data.rx_packet_count,1);
            print_int(data.rx_timeout_count,1);
            jtaguart_puts("TX:\n");
            print_int(data.tx_protocol_state,1);
            print_int(data.tx_byte_count,1);
            print_int(data.tx_packet_count,1);
            print_int(data.tx_timeout_count,1);
            print_int(data.tx_reset_count,1);
        }
        
        
    }

}
