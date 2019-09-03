
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

 #include "tools.h"


#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Set initial input parameters
#define  AFS_2G  0
#define  AFS_4G  1
#define  AFS_8G  2
#define  AFS_16G 3

#define  GFS_250DPS  0
#define  GFS_500DPS  1
#define  GFS_1000DPS 2
#define  GFS_2000DPS 3

#define  MFS_14BITS  0 // 0.6 mG per LSB
#define  MFS_16BITS  1    // 0.15 mG per LSB

#define M_8Hz   0x02
#define M_100Hz 0x06


typedef struct regs_mapping
{
    uint8_t imu_drdy;
    uint8_t imu_fsync;
    uint8_t unused[2];

    uint8_t  spi_busy;
    uint8_t  spi_en;
    uint8_t  spi_div;
    uint8_t  spi_tx_size;

    uint8_t  spi_data[16+4];

} __attribute__((packed)) regs_mapping_t;



typedef struct {
    uint8_t accel_xout_h;
    uint8_t accel_xout_l;
    uint8_t accel_yout_h;
    uint8_t accel_yout_l;
    uint8_t accel_zout_h;
    uint8_t accel_zout_l;
    uint8_t temp_out_h;
    uint8_t temp_out_l;
    uint8_t gyro_xout_h;
    uint8_t gyro_xout_l;
    uint8_t gyro_yout_h;
    uint8_t gyro_yout_l;
    uint8_t gyro_zout_h;
    uint8_t gyro_zout_l;
} __attribute__((packed)) mpu_9250_sensors_regs_t;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t temp;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
}__attribute__((packed)) mpu_9250_sensors_data_t;

typedef struct {
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
}__attribute__((packed)) mpu_9250_mag_data_t;



typedef struct {
    ProtocolHeader hdr;
    uint32_t timestamp; // 1 = 20ns
} __attribute__((packed)) serial_debug_t;


// some registers cannot be accessed at full spi speed (20mhz)
#define SPI_LOW_SPEED_DIV 64
#define SPI_HIGH_SPEED_DIV 3

void spi_init(volatile regs_mapping_t* mapping)
{
    mapping->spi_en = 0;
    mapping->spi_div = SPI_LOW_SPEED_DIV;
}

#define readBytes(x,y,z,zz) spi_read(x,y,zz,z)


void spi_write(volatile regs_mapping_t* regs, uint8_t addr, uint8_t* data, uint8_t datalen)
{
    uint8_t i;

    delay(10);
    while(regs->spi_busy);

    regs->spi_tx_size = 8+datalen*8;
    regs->spi_data[0] = addr;

    for (i=0;i<datalen;i++)
    {
        regs->spi_data[i+1] = data[i];

    }
    regs->spi_en = 1;
    delay(1);
    regs->spi_en = 0;
}

void spi_read(volatile regs_mapping_t* regs, uint8_t addr, uint8_t* data, uint8_t datalen)
{
    int i;

    delay(10);
    while(regs->spi_busy);

    regs->spi_tx_size = 8+datalen*8;
    regs->spi_data[0] = addr | 0x80;

    for (i=0;i<datalen;i++)
    {
        regs->spi_data[i+1] = 0x00;

    }
    regs->spi_en = 1;
    delay(1);
    regs->spi_en = 0;

    while(regs->spi_busy);

    for (i=0;i<datalen;i++)
    {
        data[i] = regs->spi_data[20-(datalen)+i];
    }
}


void writeByte(volatile regs_mapping_t* regs, uint8_t addr, uint8_t data)
{
    uint8_t data_local = data;
    spi_write(regs,addr,&data_local,1);
}

uint8_t readByte(volatile regs_mapping_t* regs, uint8_t addr)
{
    uint8_t ret;
    spi_read(regs,addr,&ret,1);
    return ret;
}



volatile int* pio_n = (volatile int*)(0x01000000);


typedef struct {
    ProtocolHeader hdr;
    uint64_t ts;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
}__attribute__((packed)) msg_accel_t;

typedef struct {
    ProtocolHeader hdr;
    uint64_t ts;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
}__attribute__((packed)) msg_gyro_t;


void mpu9250_init(regs_mapping_t* regs)
{
    writeByte(regs,PWR_MGMT_1,0x80); 		//Reset the sensor	
	delay(100000);
	writeByte(regs,SIGNAL_PATH_RESET,0x07);	//Reset all the digital signal path
	delay(100000);
	writeByte(regs,PWR_MGMT_1,0x01);   		//Auto select the best clock source
	writeByte(regs,PWR_MGMT_2,0x00);			//Enable all sensors
	writeByte(regs,CONFIG,0x07);				//Only effective using gyro, Set sample rate to 8 kHz
	writeByte(regs,SMPLRT_DIV,0x00);			//Only effective using gyro, Set sample rate to 8 kHz	
	writeByte(regs,GYRO_CONFIG,0x18);  		//Set gyro full range to +-2000 dps
	writeByte(regs,ACCEL_CONFIG,0x10); 		//Set Acc full range to +-8G
	writeByte(regs,ACCEL_CONFIG2,0x08);		//Bypass Acc DLPF to get 4k saple rate
    writeByte(regs, INT_PIN_CFG, 0x00);  // INT is 50 microsecond pulse and any read to clear  
	writeByte(regs,INT_ENABLE,0x01);					//Enable data ready interupt
}


uint32_t packet_count = 0;
uint32_t latest_ts = 0;
uint32_t latest_ts2 = 0;
uint32_t latest_ts_processing = 0;
uint32_t latest_ts_processing2 = 0;
uint32_t latest_ts_processing3 = 0;
uint32_t latest_ts_processing4 = 0;

void mpu9250_read_data(regs_mapping_t* regs, mpu_9250_sensors_data_t *data)
{
    uint8_t rawData[sizeof(mpu_9250_sensors_data_t)];  // x/y/z accel register data stored here

    // we use high clock speed for data only
    regs->spi_div = SPI_HIGH_SPEED_DIV;
    ts_start(&latest_ts_processing3);
    readBytes(regs, ACCEL_XOUT_H, sizeof(rawData), &rawData[0]);  // Read the 14 raw data registers into data array
    ts_stop(&latest_ts_processing3);
    uint8_t i;

    //for (i=0;i<sizeof(rawData)/2;i++)
    //    ((int16_t*)data)[i] = ((int16_t)rawData[i*2] << 8) | rawData[i*2+1] ; // reverse MSB & LSB
    ((int16_t*)data)[0] = ((int16_t)rawData[0*2] << 8) | rawData[0*2+1] ; // reverse MSB & LSB
    ((int16_t*)data)[1] = ((int16_t)rawData[1*2] << 8) | rawData[1*2+1] ; // reverse MSB & LSB
    ((int16_t*)data)[2] = ((int16_t)rawData[2*2] << 8) | rawData[2*2+1] ; // reverse MSB & LSB
    ((int16_t*)data)[3] = ((int16_t)rawData[3*2] << 8) | rawData[3*2+1] ; // reverse MSB & LSB
    ((int16_t*)data)[4] = ((int16_t)rawData[4*2] << 8) | rawData[4*2+1] ; // reverse MSB & LSB
    ((int16_t*)data)[5] = ((int16_t)rawData[5*2] << 8) | rawData[5*2+1] ; // reverse MSB & LSB
    ((int16_t*)data)[6] = ((int16_t)rawData[6*2] << 8) | rawData[6*2+1] ; // reverse MSB & LSB

    // we get back to normal speed
    regs->spi_div = SPI_LOW_SPEED_DIV;
}





void mpu9250_wait_data_ready(regs_mapping_t* regs)
{
    while (regs->imu_drdy);
    while (!regs->imu_drdy);
}

uint8_t sensors_data_identical_count = 0;   
uint32_t sensors_data_unsync_count = 0;   
mpu_9250_sensors_data_t sensors_data_previous = {0};   
mpu_9250_sensors_data_t sensors_data = {0};   

void imu_processing(regs_mapping_t* regs)
{
    static uint8_t index = 0;   
    static uint8_t imu_drdy_previous = 0;   



    uint8_t imu_drdy_current = regs->imu_drdy;

 
    if (imu_drdy_previous == 0 && imu_drdy_current == 1) 
    {
        uint64_t ts;

        ts = get_time64();

        ts_stop(&latest_ts);
        latest_ts2 = latest_ts;

        packet_count++;

        ts_start(&latest_ts_processing);
        latest_ts_processing2 = latest_ts_processing;


        //mpu_9250_sensors_data_t data;
        msg_gyro_t msg_gyro;
        msg_accel_t msg_accel;

        ts_start(&latest_ts_processing4);
        mpu9250_read_data(regs,&sensors_data);
        ts_stop(&latest_ts_processing4);

        msg_gyro.hdr.fanion = PROTOCOL_FANION;
        msg_gyro.hdr.size = sizeof(msg_gyro);
        msg_gyro.hdr.crc = 0x0000;
        msg_gyro.hdr.id = 0x0000;

        msg_gyro.ts = ts;
        msg_gyro.temp = sensors_data.temp;
        msg_gyro.gyro_x = sensors_data.gyro_x;
        msg_gyro.gyro_y = sensors_data.gyro_y;
        msg_gyro.gyro_z = sensors_data.gyro_z;


        msg_gyro.hdr.crc = protocolCrc((uint8_t*)&msg_gyro,sizeof(msg_gyro));

        msg_accel.hdr.fanion = PROTOCOL_FANION;
        msg_accel.hdr.size = sizeof(msg_accel);
        msg_accel.hdr.crc = 0x0000;
        msg_accel.hdr.id = 0x0001;

        msg_accel.ts = ts;
        msg_accel.accel_x = sensors_data.accel_x;
        msg_accel.accel_y = sensors_data.accel_y;
        msg_accel.accel_z = sensors_data.accel_z;


        msg_accel.hdr.crc = protocolCrc((uint8_t*)&msg_accel,sizeof(msg_accel));


        ts_stop(&latest_ts_processing);
        uart_rs232_tx_frame((uint8_t*)&msg_gyro,sizeof(msg_gyro));

        // Accelerometer is 4 times slower than Gyro
        if (++index == 4)
        {
            index = 0;
            uart_rs232_tx_frame((uint8_t*)&msg_accel,sizeof(msg_accel));
            if (   sensors_data_previous.accel_x == sensors_data.accel_x 
                && sensors_data_previous.accel_y == sensors_data.accel_y
                && sensors_data_previous.accel_z == sensors_data.accel_z )
            {
                sensors_data_identical_count++;

                // we consider we are unsynced after 10 identical samples
                if (sensors_data_identical_count >= 10)
                {
                    sensors_data_unsync_count++;
                    // then we shift
                    index = 1;
                    sensors_data_identical_count = 0;
                }
            } else {
                sensors_data_identical_count = 0;
            }
 
        }
        
        ts_stop(&latest_ts_processing2);

        ts_start(&latest_ts);

        sensors_data_previous = sensors_data;
    }

    imu_drdy_previous = imu_drdy_current;
}


int main()
{

    volatile regs_mapping_t* regs = (regs_mapping_t*)pio_n;
    memset((void*)regs,0,sizeof(*regs));

    jtaguart_puts("IMU Init\n");

    char chr;    


    uint8_t whoiam;
    uint8_t buf[2];
    int i;
    spi_init(regs);

    // 4.16Mbaud
    uart_rs232_configure(11);

    mpu9250_init(regs);


    for(;;) {

        imu_processing(regs);

        chr = jtaguart_getchar();
        if (chr != 0)
        {
            switch (chr)
            {
                case 'p':
                    print_int(packet_count,1);
                    print_int(latest_ts2,1);
                    print_int(latest_ts_processing,1);
                    print_int(latest_ts_processing2,1);
                    print_int(latest_ts_processing3,1);
                    print_int(latest_ts_processing4,1);
                    print_int(sensors_data_identical_count,1);
                    print_int(sensors_data_unsync_count,1);
                    break;
            }

        }

	}
}

