
#include <string.h>
#include <stdint.h>
#include <limits.h>

#include <tools.h>

#include <servo_ctrl.h>




#define RESPONSE_DELAY 50000
#define INTER_FRAME_DELAY (0)//(MS_TO_CYCLES(2))
#define RETRY_COUNT 20

uint32_t servo_ctrl_error_counter = 0;
uint32_t servo_ctrl_error_missing_response = 0;
uint32_t servo_ctrl_error_checksum_error = 0;

uint8_t servo_ctrl_error_last = 0;
uint8_t servo_ctrl_error_last_raw = 0;


uint16_t crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};


uint8_t feetech_checksum(uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t sum;

    sum = 0;
    for (i=0;i<len;i++)
    {
        sum+=data[i];
    }
    sum=(uint8_t)(~(sum&0xFF));

    return sum;
}


uint8_t feetech_msg_check(uint8_t *data, uint8_t len, uint8_t expected_len)
{
    uint8_t error = 1;

    feetech_hdr_t* msg_ptr = (feetech_hdr_t*)data;

    if (len == expected_len && msg_ptr->fanion[0] == 0xff && msg_ptr->fanion[1] == 0xff)
    {
        if (feetech_checksum((uint8_t*)&msg_ptr->id,expected_len-1-2) == data[len-1])
            error = 0;
    }
    return error;
}

uint16_t dynamixel_crc16(uint8_t *data, uint8_t len)
{
    uint16_t i, j, crc_accum;

    crc_accum = 0;

    for(j = 0; j < len; j++)
    {

        i = ((uint16_t)(crc_accum >> 8) ^ data[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}


uint8_t dynamixel_msg_check(uint8_t *data, uint8_t len, uint8_t expected_len)
{
    uint8_t error = 1;
    uint16_t crc;

    dynamixel_hdr_t* msg_ptr = (dynamixel_hdr_t*)data;

    if (len == expected_len && msg_ptr->fanion[0] == 0xff && msg_ptr->fanion[1] == 0xff && msg_ptr->fanion[2] == 0xfd)
    {
        crc = dynamixel_crc16((uint8_t*)msg_ptr,expected_len-2);

        if (((uint8_t*)&crc)[0] == data[len-2] && ((uint8_t*)&crc)[1] == data[len-1])
            error = 0;//msg_ptr->error & 0x7f;
    }
    return error;
}


uint8_t servo_ctrl_get_8b_retry(uint8_t protocol, uint8_t id, uint8_t addr, uint8_t *data_read)
{
    uint8_t retry_count = RETRY_COUNT;
    uint8_t error;

    do {
        error = servo_ctrl_get_8b(protocol,id,addr,data_read);
    } while (error == SERVO_CTRL_ERROR_BUS && --retry_count);

    servo_ctrl_error_counter+= (RETRY_COUNT-retry_count);
    
    return error;
}

dynamixel_hdr_t dynamixel_hdr = {.fanion[0] = 0xff, .fanion[1] = 0xff, .fanion[2] = 0xfd, .fanion[3] = 0x00};

uint8_t servo_ctrl_get_8b(uint8_t protocol, uint8_t id, uint8_t addr, uint8_t *data_read)
{
    uint8_t rx_buf[16];
    uint8_t rx_len;
    uint8_t error = SERVO_CTRL_ERROR_BUS;

    servo_ctrl_error_last_raw = 0;


    if (protocol == SERVO_CTRL_FEETECH)
    {
        feetech_msg_get_t msg;

        msg.hdr.fanion[0] = 0xff;
        msg.hdr.fanion[1] = 0xff;
        msg.hdr.id = id;
        msg.hdr.len = 4;
        msg.hdr.cmd = FEETECH_CMD_READ;

        msg.addr = addr;
        
        msg.datalen = 1;

        msg.checksum = feetech_checksum((uint8_t*)&msg.hdr.id,sizeof(msg)-1-2);

        uart_rs232_tx_frame((uint8_t*)&msg,sizeof(msg));
        uart_rs232_rx(NULL,0);
        rx_len = uart_rs232_rx_frame(rx_buf, RESPONSE_DELAY, 5000);


        error = feetech_msg_check(rx_buf,rx_len,sizeof(feetech_msg_get_response_8b_t));

        if (error == 0)
        {
            error = ((feetech_msg_get_response_8b_t*)rx_buf)->error;
            
            ((uint8_t*)data_read)[0] = ((feetech_msg_get_response_8b_t*)rx_buf)->data[0];

        } else {
            if (rx_len == 0)
                servo_ctrl_error_missing_response++;
            else
                servo_ctrl_error_checksum_error++;
        }

    } else {
        
        dynamixel_msg_get_t msg;

        msg.hdr = dynamixel_hdr;
        //msg.hdr.fanion[0] = 0xff;
        //msg.hdr.fanion[1] = 0xff;
        //msg.hdr.fanion[2] = 0xfd;
        //msg.hdr.fanion[3] = 0x00;

        msg.hdr.id = id;
        msg.hdr.len[0] = 0x07;
        msg.hdr.len[1] = 0x00;
        msg.hdr.cmd = DYNAMIXEL_CMD_READ;


        msg.addr[0] = addr;
        msg.addr[1] = 0x00;

        msg.datalen[0] = 1;
        msg.datalen[1] = 0x00;

        uint16_t crc = dynamixel_crc16((uint8_t*)&msg,sizeof(msg)-2);

        msg.crc[0] = ((uint8_t*)&crc)[0];
        msg.crc[1] = ((uint8_t*)&crc)[1];


        //uart_rs232_tx_frame((uint8_t*)&msg,1);
        //delay(300000);
        
        uart_rs232_tx_frame((uint8_t*)&msg,sizeof(msg));
        uart_rs232_rx(NULL,0);

        rx_len = uart_rs232_rx_frame(rx_buf, RESPONSE_DELAY, 5000);


        error = dynamixel_msg_check(rx_buf,rx_len, sizeof(dynamixel_msg_get_response_8b_t));

        if (error == 0)
        {
            //error = ((dynamixel_msg_get_response_8b_t*)rx_buf)->error & 0x7f;

            servo_ctrl_error_last_raw = ((dynamixel_msg_get_response_8b_t*)rx_buf)->error & 0x7f;
            
            switch (servo_ctrl_error_last_raw)
            {
                case DYNAMIXEL_ERROR_RESULT_FAIL:
                    error = SERVO_CTRL_ERROR_PROCESSING;
                    break;
                case DYNAMIXEL_ERROR_INSTRUCTION:
                    error = SERVO_CTRL_ERROR_PROTOCOL;
                    break;
                case DYNAMIXEL_ERROR_CRC:
                    error = SERVO_CTRL_ERROR_BUS;
                    break;
                case DYNAMIXEL_ERROR_DATA_RANGE:
                case DYNAMIXEL_ERROR_DATA_LENGTH:
                case DYNAMIXEL_ERROR_DATA_LIMIT:
                    error = SERVO_CTRL_ERROR_DATA;
                    break;
                case DYNAMIXEL_ERROR_ACCESS:
                    error = SERVO_CTRL_ERROR_ACCESS;
                    break;
            }

            *data_read = ((dynamixel_msg_get_response_8b_t*)rx_buf)->data[0];
        } else {
            if (rx_len == 0)
                servo_ctrl_error_missing_response++;
            else
                servo_ctrl_error_checksum_error++;

            error = SERVO_CTRL_ERROR_BUS;
        }

    }
    delay(INTER_FRAME_DELAY);

    servo_ctrl_error_last = error;

    return error;
}



uint8_t servo_ctrl_get_16b_retry(uint8_t protocol, uint8_t id, uint8_t addr, uint16_t *data_read)
{
    uint8_t retry_count = RETRY_COUNT;
    uint8_t error;

    do {
        error = servo_ctrl_get_16b(protocol,id,addr,data_read);
    } while (error == SERVO_CTRL_ERROR_BUS && --retry_count);

    servo_ctrl_error_counter+= (RETRY_COUNT-retry_count);
    
    return error;
}


uint8_t servo_ctrl_get_16b(uint8_t protocol, uint8_t id, uint8_t addr, uint16_t *data_read)
{
    uint8_t rx_buf[16];
    uint8_t rx_len;
    uint8_t error = SERVO_CTRL_ERROR_BUS;

    servo_ctrl_error_last_raw = 0;

    if (protocol == SERVO_CTRL_FEETECH)
    {
        feetech_msg_get_t msg;

        msg.hdr.fanion[0] = 0xff;
        msg.hdr.fanion[1] = 0xff;
        msg.hdr.id = id;
        msg.hdr.len = 4;
        msg.hdr.cmd = FEETECH_CMD_READ;

        msg.addr = addr;
        
        msg.datalen = 2;

        msg.checksum = feetech_checksum((uint8_t*)&msg.hdr.id,sizeof(msg)-1-2);

        uart_rs232_tx_frame((uint8_t*)&msg,sizeof(msg));
        uart_rs232_rx(NULL,0);
        rx_len = uart_rs232_rx_frame(rx_buf, RESPONSE_DELAY, 5000);

        error = feetech_msg_check(rx_buf,rx_len,sizeof(feetech_msg_get_response_t));

        if (error == 0)
        {
            error = ((feetech_msg_get_response_t*)rx_buf)->error;

            ((uint8_t*)data_read)[0] = ((feetech_msg_get_response_t*)rx_buf)->data[1];
            ((uint8_t*)data_read)[1] = ((feetech_msg_get_response_t*)rx_buf)->data[0]; 
        } else {
            if (rx_len == 0)
                servo_ctrl_error_missing_response++;
            else
                servo_ctrl_error_checksum_error++;
        }

    } else {
        
        dynamixel_msg_get_t msg;

        msg.hdr = dynamixel_hdr;
/*
        msg.hdr.fanion[0] = 0xff;
        msg.hdr.fanion[1] = 0xff;
        msg.hdr.fanion[2] = 0xfd;
        msg.hdr.fanion[3] = 0x00;
*/
        msg.hdr.id = id;
        msg.hdr.len[0] = 0x07;
        msg.hdr.len[1] = 0x00;
        msg.hdr.cmd = DYNAMIXEL_CMD_READ;


        msg.addr[0] = addr;
        msg.addr[1] = 0x00;

        msg.datalen[0] = 2;
        msg.datalen[1] = 0x00;

        uint16_t crc = dynamixel_crc16((uint8_t*)&msg,sizeof(msg)-2);

        msg.crc[0] = ((uint8_t*)&crc)[0];
        msg.crc[1] = ((uint8_t*)&crc)[1];


        //uart_rs232_tx_frame((uint8_t*)&msg,1);
        //delay(300000);
        
        uart_rs232_tx_frame((uint8_t*)&msg,sizeof(msg));
        uart_rs232_rx(NULL,0);

        rx_len = uart_rs232_rx_frame(rx_buf, RESPONSE_DELAY, 5000);


        error = dynamixel_msg_check(rx_buf,rx_len, sizeof(dynamixel_msg_get_response_t));
/*
        uint8_t i;

        for (i=0;i<rx_len;i++)
        {
            print_int(rx_buf[i],1);
        }
*/
        if (error == 0)
        {
            //error = ((dynamixel_msg_get_response_t*)rx_buf)->error & 0x7f;


            servo_ctrl_error_last_raw = ((dynamixel_msg_get_response_t*)rx_buf)->error & 0x7f;
            
            switch (servo_ctrl_error_last_raw)
            {
                case DYNAMIXEL_ERROR_RESULT_FAIL:
                    error = SERVO_CTRL_ERROR_PROCESSING;
                    break;
                case DYNAMIXEL_ERROR_INSTRUCTION:
                    error = SERVO_CTRL_ERROR_PROTOCOL;
                    break;
                case DYNAMIXEL_ERROR_CRC:
                    error = SERVO_CTRL_ERROR_BUS;
                    break;
                case DYNAMIXEL_ERROR_DATA_RANGE:
                case DYNAMIXEL_ERROR_DATA_LENGTH:
                case DYNAMIXEL_ERROR_DATA_LIMIT:
                    error = SERVO_CTRL_ERROR_DATA;
                    break;
                case DYNAMIXEL_ERROR_ACCESS:
                    error = SERVO_CTRL_ERROR_ACCESS;
                    break;
            }


            ((uint8_t*)data_read)[0] = ((dynamixel_msg_get_response_t*)rx_buf)->data[0];
            ((uint8_t*)data_read)[1] = ((dynamixel_msg_get_response_t*)rx_buf)->data[1]; 
        } else {
            if (rx_len == 0)
                servo_ctrl_error_missing_response++;
            else
                servo_ctrl_error_checksum_error++;

            error = SERVO_CTRL_ERROR_BUS;
        }

    }
    delay(INTER_FRAME_DELAY);
    servo_ctrl_error_last = error;
    return error;
}



uint8_t servo_ctrl_set_8b_retry(uint8_t protocol, uint8_t on_hold, uint8_t id, uint8_t addr,  uint8_t data)
{
    uint8_t retry_count = RETRY_COUNT;
    uint8_t error;

    do {
        error = servo_ctrl_set_8b(protocol,on_hold,id,addr,data);
    } while (error == SERVO_CTRL_ERROR_BUS && --retry_count);

    servo_ctrl_error_counter += (RETRY_COUNT-retry_count);
    
    return error;
}



uint8_t servo_ctrl_set_8b(uint8_t protocol, uint8_t on_hold, uint8_t id, uint8_t addr, uint8_t data)
{
    uint8_t rx_buf[16];
    uint8_t rx_len;
    uint8_t error = SERVO_CTRL_ERROR_BUS;

    servo_ctrl_error_last_raw = 0;

    if (protocol == SERVO_CTRL_FEETECH)
    {
        feetech_msg_set8b_t msg;

        msg.hdr.fanion[0] = 0xff;
        msg.hdr.fanion[1] = 0xff;
        msg.hdr.id = id;
        msg.hdr.len = 4;
        if (on_hold == 0)
            msg.hdr.cmd = FEETECH_CMD_WRITE;
        else
            msg.hdr.cmd = FEETECH_CMD_REG_WRITE;

        msg.addr = addr;
        
        msg.data[0] = data;

        msg.checksum = feetech_checksum((uint8_t*)&msg.hdr.id,sizeof(msg)-1-2);

        uart_rs232_tx_frame((uint8_t*)&msg,sizeof(msg));
        uart_rs232_rx(NULL,0);

        rx_len = uart_rs232_rx_frame(rx_buf, RESPONSE_DELAY, 5000);
        error = feetech_msg_check(rx_buf,rx_len,sizeof(feetech_msg_status_t));

        if (error == 0)
            error = ((feetech_msg_status_t*)rx_buf)->error;
        else {
            if (rx_len == 0)
                servo_ctrl_error_missing_response++;
            else
                servo_ctrl_error_checksum_error++;
        }

    } else {
        
        dynamixel_msg_set8b_t msg;

        msg.hdr = dynamixel_hdr;
/*
        msg.hdr.fanion[0] = 0xff;
        msg.hdr.fanion[1] = 0xff;
        msg.hdr.fanion[2] = 0xfd;
        msg.hdr.fanion[3] = 0x00;
*/
        msg.hdr.id = id;
        msg.hdr.len[0] = 0x06;
        msg.hdr.len[1] = 0x00;
        if (on_hold == 0)
            msg.hdr.cmd = DYNAMIXEL_CMD_WRITE;
        else
            msg.hdr.cmd = DYNAMIXEL_CMD_REG_WRITE;


        msg.addr[0] = addr;
        msg.addr[1] = 0x00;

        msg.data[0] = data;

        uint16_t crc = dynamixel_crc16((uint8_t*)&msg,sizeof(msg)-2);

        msg.crc[0] = ((uint8_t*)&crc)[0];
        msg.crc[1] = ((uint8_t*)&crc)[1];


        uart_rs232_tx_frame((uint8_t*)&msg,sizeof(msg));
        uart_rs232_rx(NULL,0);

        rx_len = uart_rs232_rx_frame(rx_buf, RESPONSE_DELAY, 5000);
        error = dynamixel_msg_check(rx_buf,rx_len,sizeof(dynamixel_msg_status_t));

        if (error == 0)
        {
            //error = ((dynamixel_msg_status_t*)rx_buf)->error & 0x7f;


            servo_ctrl_error_last_raw = ((dynamixel_msg_status_t*)rx_buf)->error & 0x7f;
            
            switch (servo_ctrl_error_last_raw)
            {
                case DYNAMIXEL_ERROR_RESULT_FAIL:
                    error = SERVO_CTRL_ERROR_PROCESSING;
                    break;
                case DYNAMIXEL_ERROR_INSTRUCTION:
                    error = SERVO_CTRL_ERROR_PROTOCOL;
                    break;
                case DYNAMIXEL_ERROR_CRC:
                    error = SERVO_CTRL_ERROR_BUS;
                    break;
                case DYNAMIXEL_ERROR_DATA_RANGE:
                case DYNAMIXEL_ERROR_DATA_LENGTH:
                case DYNAMIXEL_ERROR_DATA_LIMIT:
                    error = SERVO_CTRL_ERROR_DATA;
                    break;
                case DYNAMIXEL_ERROR_ACCESS:
                    error = SERVO_CTRL_ERROR_ACCESS;
                    break;
            }
        } else {
            if (rx_len == 0)
                servo_ctrl_error_missing_response++;
            else
                servo_ctrl_error_checksum_error++;

            error = SERVO_CTRL_ERROR_BUS;
        }

    }
    delay(INTER_FRAME_DELAY);
    servo_ctrl_error_last = error;
    return error;
}

uint8_t servo_ctrl_set_16b_retry(uint8_t protocol, uint8_t on_hold, uint8_t id, uint8_t addr,  uint16_t data)
{
    uint8_t retry_count = RETRY_COUNT;
    uint8_t error;

    do {
        error = servo_ctrl_set_16b(protocol,on_hold,id,addr,data);
    } while (error == SERVO_CTRL_ERROR_BUS && --retry_count);

    servo_ctrl_error_counter+= (RETRY_COUNT-retry_count);
    
    return error;
}


uint8_t servo_ctrl_set_16b(uint8_t protocol, uint8_t on_hold, uint8_t id, uint8_t addr,  uint16_t data)
{
    uint8_t rx_buf[16];
    uint8_t rx_len;
    uint8_t error = SERVO_CTRL_ERROR_BUS;

    servo_ctrl_error_last_raw = 0;

    if (protocol == SERVO_CTRL_FEETECH)
    {
        feetech_msg_set16b_t msg;

        msg.hdr.fanion[0] = 0xff;
        msg.hdr.fanion[1] = 0xff;
        msg.hdr.id = id;
        msg.hdr.len = 3+2;
        if (on_hold == 0)
            msg.hdr.cmd = FEETECH_CMD_WRITE;
        else
            msg.hdr.cmd = FEETECH_CMD_REG_WRITE;

        msg.addr = addr;
        
        msg.data[0] = ((uint8_t*)&data)[1];
        msg.data[1] = ((uint8_t*)&data)[0];

        msg.checksum = feetech_checksum((uint8_t*)&msg.hdr.id,sizeof(msg)-1-2);

        uart_rs232_tx_frame((uint8_t*)&msg,sizeof(msg));
        uart_rs232_rx(NULL,0);
        rx_len = uart_rs232_rx_frame(rx_buf, RESPONSE_DELAY, 5000);

        error = feetech_msg_check(rx_buf,rx_len,sizeof(feetech_msg_status_t));

        if (error == 0)
            error = ((feetech_msg_status_t*)rx_buf)->error;
        else {
            if (rx_len == 0)
                servo_ctrl_error_missing_response++;
            else
                servo_ctrl_error_checksum_error++;
        }

    } else {
        
        dynamixel_msg_set16b_t msg;

        msg.hdr = dynamixel_hdr;
/*
        msg.hdr.fanion[0] = 0xff;
        msg.hdr.fanion[1] = 0xff;
        msg.hdr.fanion[2] = 0xfd;
        msg.hdr.fanion[3] = 0x00;
*/
        msg.hdr.id = id;
        msg.hdr.len[0] = 0x07;
        msg.hdr.len[1] = 0x00;
        if (on_hold == 0)
            msg.hdr.cmd = DYNAMIXEL_CMD_WRITE;
        else
            msg.hdr.cmd = DYNAMIXEL_CMD_REG_WRITE;


        msg.addr[0] = addr;
        msg.addr[1] = 0x00;

        msg.data[0] = ((uint8_t*)&data)[0];
        msg.data[1] = ((uint8_t*)&data)[1];

        uint16_t crc = dynamixel_crc16((uint8_t*)&msg,sizeof(msg)-2);

        msg.crc[0] = ((uint8_t*)&crc)[0];
        msg.crc[1] = ((uint8_t*)&crc)[1];


        //uart_rs232_tx_frame((uint8_t*)&msg,1);
        //delay(200000);

        uart_rs232_tx_frame((uint8_t*)&msg,sizeof(msg));
        uart_rs232_rx(NULL,0);

        rx_len = uart_rs232_rx_frame(rx_buf, RESPONSE_DELAY, 5000);


        error = dynamixel_msg_check(rx_buf,rx_len,sizeof(dynamixel_msg_status_t));

        if (error == 0)
        {
            //error = ((dynamixel_msg_status_t*)rx_buf)->error & 0x7f;

            servo_ctrl_error_last_raw = ((dynamixel_msg_status_t*)rx_buf)->error & 0x7f;

            switch (servo_ctrl_error_last_raw)
            {
                case DYNAMIXEL_ERROR_RESULT_FAIL:
                    error = SERVO_CTRL_ERROR_PROCESSING;
                    break;
                case DYNAMIXEL_ERROR_INSTRUCTION:
                    error = SERVO_CTRL_ERROR_PROTOCOL;
                    break;
                case DYNAMIXEL_ERROR_CRC:
                    error = SERVO_CTRL_ERROR_BUS;
                    break;
                case DYNAMIXEL_ERROR_DATA_RANGE:
                case DYNAMIXEL_ERROR_DATA_LENGTH:
                case DYNAMIXEL_ERROR_DATA_LIMIT:
                    error = SERVO_CTRL_ERROR_DATA;
                    break;
                case DYNAMIXEL_ERROR_ACCESS:
                    error = SERVO_CTRL_ERROR_ACCESS;
                    break;
            }
            
        } else {
            if (rx_len == 0)
                servo_ctrl_error_missing_response++;
            else
                servo_ctrl_error_checksum_error++;

            error = SERVO_CTRL_ERROR_BUS;
        }
    }
    //delay(5000);
    delay(INTER_FRAME_DELAY);
    servo_ctrl_error_last = error;

    return error;
}

uint8_t servo_ctrl_set_action(uint8_t protocol)
{
    if (protocol == SERVO_CTRL_FEETECH)
    {
        feetech_msg_action_t msg;

        msg.hdr.fanion[0] = 0xff;
        msg.hdr.fanion[1] = 0xff;
        msg.hdr.id = 0xFE;
        msg.hdr.len = 2;

        msg.hdr.cmd = FEETECH_CMD_ACTION;

        msg.checksum = feetech_checksum((uint8_t*)&msg.hdr.id,sizeof(msg)-1-2);

        uart_rs232_tx_frame((uint8_t*)&msg,sizeof(msg));
        uart_rs232_rx(NULL,0);

    } else {
        
        dynamixel_msg_action_t msg;

        msg.hdr = dynamixel_hdr;
/*
        msg.hdr.fanion[0] = 0xff;
        msg.hdr.fanion[1] = 0xff;
        msg.hdr.fanion[2] = 0xfd;
        msg.hdr.fanion[3] = 0x00;
*/
        msg.hdr.id = 0xFE;
        msg.hdr.len[0] = 0x03;
        msg.hdr.len[1] = 0x00;
        msg.hdr.cmd = DYNAMIXEL_CMD_ACTION;

        uint16_t crc = dynamixel_crc16((uint8_t*)&msg,sizeof(msg)-2);

        msg.crc[0] = ((uint8_t*)&crc)[0];
        msg.crc[1] = ((uint8_t*)&crc)[1];


        uart_rs232_tx_frame((uint8_t*)&msg,sizeof(msg));
        uart_rs232_rx(NULL,0);

    }
    delay(5000);
    return 0;


}


uint8_t servo_ctrl_set_pos(uint8_t protocol,uint8_t on_hold, uint8_t id, uint16_t pos)
{
    if (protocol == SERVO_CTRL_FEETECH)
    {
        return servo_ctrl_set_16b_retry(SERVO_CTRL_FEETECH,on_hold,id,FEETECH_REGS_TARGET_POS_H,pos);
    }
    else
    {
        return servo_ctrl_set_16b_retry(SERVO_CTRL_DYNAMIXEL,on_hold,id,DYNAMIXEL_REGS_GOAL_POSITION_L,pos);
    }
}

uint8_t servo_ctrl_set_speed(uint8_t protocol,uint8_t on_hold, uint8_t id, uint16_t speed)
{
    if (protocol == SERVO_CTRL_FEETECH)
    {
        return servo_ctrl_set_16b_retry(SERVO_CTRL_FEETECH,on_hold,id,FEETECH_REGS_RUNNING_SPEED_H,speed);
    }
    else
    {
        return servo_ctrl_set_16b_retry(SERVO_CTRL_DYNAMIXEL,on_hold,id,DYNAMIXEL_REGS_GOAL_VELOCITY_L,speed);
    }
}

uint8_t servo_ctrl_get_pos(uint8_t protocol, uint8_t id, uint16_t *data_read)
{
    if (protocol == SERVO_CTRL_FEETECH)
    {
        return servo_ctrl_get_16b_retry(SERVO_CTRL_FEETECH,id,FEETECH_REGS_CURRENT_POS_H,data_read);
    }
    else
    {
        return servo_ctrl_get_16b_retry(SERVO_CTRL_DYNAMIXEL,id,DYNAMIXEL_REGS_PRESENT_POSITION_L,data_read);
    }
}

uint8_t servo_ctrl_get_status(uint8_t protocol, uint8_t id, uint16_t *pos, uint16_t *load, uint8_t *voltage, uint8_t *temp)
{
    uint8_t error = 0;

    error = servo_ctrl_get_pos(protocol,id,pos);

    if (protocol == SERVO_CTRL_FEETECH)
    {
        if (!error)
            error = servo_ctrl_get_16b_retry(SERVO_CTRL_FEETECH,id,FEETECH_REGS_CURRENT_LOAD_H,load);
        delay(MS_TO_CYCLES(1));
        if (!error)
            error = servo_ctrl_get_8b_retry(SERVO_CTRL_FEETECH,id,FEETECH_REGS_CURRENT_VOLTAGE,voltage);
        delay(MS_TO_CYCLES(1));
        if (!error)
            error = servo_ctrl_get_8b_retry(SERVO_CTRL_FEETECH,id,FEETECH_REGS_CURRENT_TEMP,temp);
    }
    else
    {
        if (!error)
            error = servo_ctrl_get_16b_retry(SERVO_CTRL_DYNAMIXEL,id,DYNAMIXEL_REGS_PRESENT_LOAD_L,load);
        delay(MS_TO_CYCLES(1));
        if (!error)
            error = servo_ctrl_get_8b_retry(SERVO_CTRL_DYNAMIXEL,id,DYNAMIXEL_REGS_PRESENT_VOLTAGE,voltage);
        delay(MS_TO_CYCLES(1));
        if (!error)
            error = servo_ctrl_get_8b_retry(SERVO_CTRL_DYNAMIXEL,id,DYNAMIXEL_REGS_PRESENT_TEMP,temp);
    }
    return error;

}

uint8_t servo_ctrl_set_enable(uint8_t protocol,uint8_t on_hold, uint8_t id, uint8_t enable)
{
    uint8_t error;
    if (protocol == SERVO_CTRL_FEETECH)
    { 
        error = servo_ctrl_set_8b_retry(SERVO_CTRL_FEETECH,on_hold,id,FEETECH_REGS_TORQUE_SWITCH,enable);
    }
    else
    {   
        error = servo_ctrl_set_8b_retry(SERVO_CTRL_DYNAMIXEL,on_hold,id,DYNAMIXEL_REGS_TORQUE_ENABLE,enable);
        if (enable && !error)
            error += servo_ctrl_set_16b_retry(SERVO_CTRL_DYNAMIXEL,on_hold,id,DYNAMIXEL_REGS_GOAL_TORQUE_L,1023);
    }
    return error;
}

inline uint8_t servo_ctrl_get_last_error()
{
    return servo_ctrl_error_last;
}

inline uint8_t servo_ctrl_get_last_error_raw()
{
    return servo_ctrl_error_last_raw;
}


