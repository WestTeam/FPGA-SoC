
#include <string.h>
#include <stdint.h>
#include <limits.h>

//#include <tools.h>

#define SERVO_CTRL_FEETECH 0
#define SERVO_CTRL_DYNAMIXEL 1


//FEETECH : Definition of commands
#define FEETECH_CMD_PING        0x01
#define FEETECH_CMD_READ        0x02
#define FEETECH_CMD_WRITE       0x03
#define FEETECH_CMD_REG_WRITE   0x04
#define FEETECH_CMD_ACTION      0x05
#define FEETECH_CMD_RESET       0x06
#define FEETECH_CMD_SYNC_WRITE  0x83

//FEETECH : Definition of regs
// EEPROM
#define FEETECH_REGS_ID 0x05
#define FEETECH_REGS_BAUD_RATE 0x06
#define FEETECH_REGS_RETURN_DELAY_TIME 0x07
#define FEETECH_REGS_ANSWER_LEVEL_STATUS 0x08
#define FEETECH_REGS_MIN_ANGLE_LIMIT_H 0x09

#define FEETECH_REGS_MAX_ANGLE_LIMIT_H 0x0b

#define FEETECH_REGS_MAX_TEMP 0x0d
#define FEETECH_REGS_HIGHEST_VOLTAGE 0x0e
#define FEETECH_REGS_LOWEST_VOLTAGE 0x0f
#define FEETECH_REGS_MAX_TORQUE_H 0x10

#define FEETECH_REGS_HIGH_VOLTAGE_FLAG 0x12
#define FEETECH_REGS_UNLOAD_CONDITION 0x13
#define FEETECH_REGS_LED_ALARAM_CONDITION 0x14
#define FEETECH_REGS_P 0x15
#define FEETECH_REGS_I 0x16
#define FEETECH_REGS_D 0x17
#define FEETECH_REGS_MIN_PWM_H 0x18


// RAM
#define FEETECH_REGS_TORQUE_SWITCH 0x28
#define FEETECH_REGS_TARGET_POS_H 0x2a
#define FEETECH_REGS_RUNNING_TIME_H 0x2c
#define FEETECH_REGS_RUNNING_SPEED_H 0x2e
#define FEETECH_REGS_LOCK_SIGN 0x30

#define FEETECH_REGS_CURRENT_POS_H 0x38
#define FEETECH_REGS_CURRENT_SPEED_H 0x3a
#define FEETECH_REGS_CURRENT_LOAD_H 0x3c
#define FEETECH_REGS_CURRENT_VOLTAGE 0x3e
#define FEETECH_REGS_CURRENT_TEMP 0x3f
#define FEETECH_REGS_REG_WRITE_SIGN 0x40

//DYNAMIXEL : Definition of commands
#define DYNAMIXEL_CMD_PING          0x01
#define DYNAMIXEL_CMD_READ          0x02
#define DYNAMIXEL_CMD_WRITE         0x03
#define DYNAMIXEL_CMD_REG_WRITE     0x04
#define DYNAMIXEL_CMD_ACTION        0x05
#define DYNAMIXEL_CMD_FACTORY_RESET 0x06
#define DYNAMIXEL_CMD_REBOOT        0x08
#define DYNAMIXEL_CMD_STATUS        0x55
#define DYNAMIXEL_CMD_SYNC_READ     0x82
#define DYNAMIXEL_CMD_SYNC_WRITE    0x83
#define DYNAMIXEL_CMD_BULK_READ     0x92
#define DYNAMIXEL_CMD_BULK_WRITE    0x93

//DYNAMIXEL : Definition of regs
// EEPROM
#define DYNAMIXEL_REGS_ID (3)
#define DYNAMIXEL_REGS_BAUD_RATE (4)
#define DYNAMIXEL_REGS_RETURN_DELAY_TIME (5)
#define DYNAMIXEL_REGS_CW_ANGLE_LIMIT_L (6)
#define DYNAMIXEL_REGS_CCW_ANGLE_LIMIT_L (8)
#define DYNAMIXEL_REGS_CONTROL_MODE (11)
#define DYNAMIXEL_REGS_LIMIT_TEMP (12)
#define DYNAMIXEL_REGS_LOWER_LIMIT_VOLTAGE (13)
#define DYNAMIXEL_REGS_UPPER_LIMIT_VOLTAGE (14)
#define DYNAMIXEL_REGS_MAX_TORQUE_L (15)
#define DYNAMIXEL_REGS_RETURN_LEVEL (17)
#define DYNAMIXEL_REGS_ALARM_SHUTDOWN (18)


// RAM
#define DYNAMIXEL_REGS_TORQUE_ENABLE (24)
#define DYNAMIXEL_REGS_LED (25)
#define DYNAMIXEL_REGS_D (27)
#define DYNAMIXEL_REGS_I (28)
#define DYNAMIXEL_REGS_P (29)
#define DYNAMIXEL_REGS_GOAL_POSITION_L (30)
#define DYNAMIXEL_REGS_GOAL_VELOCITY_L (32)
#define DYNAMIXEL_REGS_GOAL_TORQUE_L (35)
#define DYNAMIXEL_REGS_PRESENT_POSITION_L (37)
#define DYNAMIXEL_REGS_PRESENT_SPEED_L (39)
#define DYNAMIXEL_REGS_PRESENT_LOAD_L (41)
#define DYNAMIXEL_REGS_PRESENT_VOLTAGE (45)
#define DYNAMIXEL_REGS_PRESENT_TEMP (46)
#define DYNAMIXEL_REGS_REGISTERED_INSTR (47)
#define DYNAMIXEL_REGS_MOVING (49)
#define DYNAMIXEL_REGS_HW_ERROR_STATUS (50)
#define DYNAMIXEL_REGS_PUNCH (51)

#define DYNAMIXEL_ERROR_RESULT_FAIL (0x01)
#define DYNAMIXEL_ERROR_INSTRUCTION (0x02)
#define DYNAMIXEL_ERROR_CRC (0x03)
#define DYNAMIXEL_ERROR_DATA_RANGE (0x04)
#define DYNAMIXEL_ERROR_DATA_LENGTH (0x05)
#define DYNAMIXEL_ERROR_DATA_LIMIT (0x06)
#define DYNAMIXEL_ERROR_ACCESS (0x07)

extern uint32_t servo_ctrl_error_counter;
extern uint32_t servo_ctrl_error_missing_response;
extern uint32_t servo_ctrl_error_checksum_error;

extern uint8_t servo_ctrl_error_last;
extern uint8_t servo_ctrl_error_last_raw;

#define SERVO_CTRL_SUCCESS (0)
#define SERVO_CTRL_ERROR (1)


#define SERVO_CTRL_ERROR_BUS (1) // missing response or crc error
#define SERVO_CTRL_ERROR_PROTOCOL (2) // unknown instruction or no instruction to do
#define SERVO_CTRL_ERROR_DATA (3) // wrong data (range, length, limit)
#define SERVO_CTRL_ERROR_ACCESS (4) // wrong access (wrongly accessed register)
#define SERVO_CTRL_ERROR_PROCESSING (5) // error on the processing of the command
#define SERVO_CTRL_ERROR_TIMEOUT (6) // timeout occured when polling on position target



typedef struct {
    uint8_t fanion[2];
    uint8_t id;
    uint8_t len;
    uint8_t cmd;
} __attribute__((packed)) feetech_hdr_t;


typedef struct {
    uint8_t fanion[2];
    uint8_t id;
    uint8_t len;
    uint8_t error;
    uint8_t checksum;
} __attribute__((packed)) feetech_msg_status_t;


typedef struct {
    feetech_hdr_t hdr;
    uint8_t addr;
    uint8_t data[1];
    uint8_t checksum;

} __attribute__((packed)) feetech_msg_set8b_t;

typedef struct {
    feetech_hdr_t hdr;
    uint8_t checksum;
} __attribute__((packed)) feetech_msg_action_t;

typedef struct {
    feetech_hdr_t hdr;
    uint8_t addr;
    uint8_t data[2];
    uint8_t checksum;

} __attribute__((packed)) feetech_msg_set16b_t;

typedef struct {
    feetech_hdr_t hdr;
    uint8_t addr;
    uint8_t datalen;
    uint8_t checksum;
} __attribute__((packed)) feetech_msg_get_t;

typedef struct {
    uint8_t fanion[2];
    uint8_t id;
    uint8_t len;
    uint8_t error;
    uint8_t data[2];
    uint8_t checksum;
} __attribute__((packed)) feetech_msg_get_response_t;

typedef struct {
    uint8_t fanion[2];
    uint8_t id;
    uint8_t len;
    uint8_t error;
    uint8_t data[1];
    uint8_t checksum;
} __attribute__((packed)) feetech_msg_get_response_8b_t;




typedef struct {
    uint8_t fanion[4];
    uint8_t id;
    uint8_t len[2];
    uint8_t cmd;
} __attribute__((packed)) dynamixel_hdr_t;



typedef struct {
    uint8_t fanion[4];
    uint8_t id;
    uint8_t len[2];
    uint8_t cmd;
    uint8_t error;
    uint8_t crc[2];
} __attribute__((packed)) dynamixel_msg_status_t;


typedef struct {
    dynamixel_hdr_t hdr;
    uint8_t addr[2];
    uint8_t data[1];
    uint8_t crc[2];
} __attribute__((packed)) dynamixel_msg_set8b_t;


typedef struct {
    dynamixel_hdr_t hdr;
    uint8_t crc[2];
} __attribute__((packed)) dynamixel_msg_action_t;



typedef struct {
    dynamixel_hdr_t hdr;
    uint8_t addr[2];
    uint8_t data[2];
    uint8_t crc[2];
} __attribute__((packed)) dynamixel_msg_set16b_t;


typedef struct {
    dynamixel_hdr_t hdr;
    uint8_t addr[2];
    uint8_t datalen[2];
    uint8_t crc[2];
} __attribute__((packed)) dynamixel_msg_get_t;

typedef struct {
    uint8_t fanion[4];
    uint8_t id;
    uint8_t len[2];
    uint8_t cmd;
    uint8_t error;
    uint8_t data[2];
    uint8_t crc[2];
} __attribute__((packed)) dynamixel_msg_get_response_t;

typedef struct {
    uint8_t fanion[4];
    uint8_t id;
    uint8_t len[2];
    uint8_t cmd;
    uint8_t error;
    uint8_t data[1];
    uint8_t crc[2];
} __attribute__((packed)) dynamixel_msg_get_response_8b_t;


uint8_t servo_ctrl_set_8b_retry(uint8_t protocol, uint8_t on_hold, uint8_t id, uint8_t addr,  uint8_t data);
uint8_t servo_ctrl_set_16b_retry(uint8_t protocol, uint8_t on_hold, uint8_t id, uint8_t addr,  uint16_t data);

uint8_t servo_ctrl_set_16b(uint8_t protocol, uint8_t on_hold, uint8_t id, uint8_t addr,  uint16_t data);
uint8_t servo_ctrl_set_8b(uint8_t protocol, uint8_t on_hold, uint8_t id, uint8_t addr, uint8_t data);

uint8_t servo_ctrl_get_8b_retry(uint8_t protocol, uint8_t id, uint8_t addr, uint8_t *data_read);
uint8_t servo_ctrl_get_8b(uint8_t protocol, uint8_t id, uint8_t addr, uint8_t *data_read);

uint8_t servo_ctrl_get_16b_retry(uint8_t protocol, uint8_t id, uint8_t addr, uint16_t *data_read);
uint8_t servo_ctrl_get_16b(uint8_t protocol, uint8_t id, uint8_t addr, uint16_t *data_read);

uint8_t servo_ctrl_set_action(uint8_t protocol);

uint8_t servo_ctrl_set_pos(uint8_t protocol,uint8_t on_hold, uint8_t id, uint16_t pos);
uint8_t servo_ctrl_set_speed(uint8_t protocol,uint8_t on_hold, uint8_t id, uint16_t speed);

uint8_t servo_ctrl_get_pos(uint8_t protocol, uint8_t id, uint16_t *data_read);
uint8_t servo_ctrl_get_status(uint8_t protocol, uint8_t id, uint16_t *pos, uint16_t *load, uint8_t *voltage, uint8_t *temp);

uint8_t servo_ctrl_set_enable(uint8_t protocol,uint8_t on_hold, uint8_t id, uint8_t enable);

uint8_t feetech_checksum(uint8_t *data, uint8_t len);
uint16_t dynamixel_crc16(uint8_t *data, uint8_t len);


uint8_t servo_ctrl_get_last_error();

uint8_t servo_ctrl_get_last_error_raw();
