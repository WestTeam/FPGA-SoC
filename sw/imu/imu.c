
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

 #include "tools.h"


//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2     0x0B  // Reset
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

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

// Define I2C addresses of the two MPU9250
#define MPU9250_1_ADDRESS 0x68   // Device address when ADO = 0
#define MPU1              0x68
#define MPU9250_2_ADDRESS 0x69   // Device address when ADO = 1
#define MPU2              0x69
#define AK8963_ADDRESS    0x0C   //  Address of magnetometer


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


void spi_init(volatile regs_mapping_t* mapping)
{
    mapping->spi_en = 0;
    mapping->spi_div = 64;
}


void spi_write(volatile regs_mapping_t* mapping, uint8_t addr, uint8_t* data, uint8_t datalen)
{
    int i;

    delay(10);
    while(mapping->spi_busy);

    mapping->spi_tx_size = 8+datalen*8;
    mapping->spi_data[0] = addr;

    for (i=0;i<datalen;i++)
    {
        mapping->spi_data[i+1] = data[i];

    }
    mapping->spi_en = 1;
    delay(1);
    mapping->spi_en = 0;
}

void spi_read(volatile regs_mapping_t* mapping, uint8_t addr, uint8_t* data, uint8_t datalen)
{
    int i;

    delay(10);
    while(mapping->spi_busy);

    mapping->spi_tx_size = 8+datalen*8;
    mapping->spi_data[0] = addr | 0x80;

    for (i=0;i<datalen;i++)
    {
        mapping->spi_data[i+1] = 0x00;

    }
    mapping->spi_en = 1;
    delay(1);
    mapping->spi_en = 0;

    while(mapping->spi_busy);

    for (i=0;i<datalen;i++)
    {
        data[i] = mapping->spi_data[20-(datalen)+i];
    }
}


void writeByte(volatile regs_mapping_t* MPUnum, uint8_t addr, uint8_t data)
{
    uint8_t data_local = data;
    spi_write(MPUnum,addr,&data_local,1);
}

uint8_t readByte(volatile regs_mapping_t* MPUnum, uint8_t addr)
{
    uint8_t ret;
    spi_read(MPUnum,addr,&ret,1);
    return ret;
}



void MPU9250_initMPU9250(volatile regs_mapping_t* MPUnum, uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate)
{  
 // wake up device
  writeByte(MPUnum, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(MPUnum, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 


 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPUnum, CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPUnum, SMPLRT_DIV, sampleRate);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                                       // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPUnum, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x02; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPUnum, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
  
 // Set accelerometer full-scale range configuration
  c = readByte(MPUnum, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
  writeByte(MPUnum, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPUnum, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPUnum, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPUnum, INT_PIN_CFG, 0x10);  // INT is 50 microsecond pulse and any read to clear  
   writeByte(MPUnum, INT_ENABLE, 0x01);   // Enable data ready (bit 0) interrupt
   delay(100);

  writeByte(MPUnum, USER_CTRL, 0x20);          // Enable I2C Master mode  
  writeByte(MPUnum, I2C_MST_CTRL, 0x1D);       // I2C configuration STOP after each transaction, master I2C bus at 400 KHz
  writeByte(MPUnum, I2C_MST_DELAY_CTRL, 0x00); // Use blocking data retreival and enable delay for mag sample rate mismatch
  writeByte(MPUnum, I2C_SLV4_CTRL, 0x00);      // Delay mag data retrieval to once every other accel/gyro data sample
}


#define readBytes(x,y,z,zz) spi_read(x,y,zz,z)


  uint8_t MPU9250_getAK8963CID(volatile regs_mapping_t* MPUnum)
{
//  uint8_t c = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for MPU-9250
  writeByte(MPUnum, USER_CTRL, 0x20);    // Enable I2C Master mode  
  writeByte(MPUnum, I2C_MST_CTRL, 0x00); // I2C configuration multi-master I2C 400KHz

  writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
  writeByte(MPUnum, I2C_SLV4_REG, WHO_AM_I_AK8963);           // I2C slave 0 register address from where to begin data transfer
  writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and transfer 1 byte
  delay(100000);
  uint8_t c = readByte(MPUnum, I2C_SLV4_DI);             // Read the WHO_AM_I byte
  return c;
}


void MPU9250_initAK8963Slave(volatile regs_mapping_t* MPUnum, uint8_t Mscale, uint8_t Mmode, float * magCalibration)
{
   // First extract the factory calibration for each magnetometer axis
   uint8_t rawData[3];  // x/y/z gyro calibration data stored here
//   _Mmode = Mmode;

   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_CNTL2);              // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_DO, 0x01);                       // Reset AK8963
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and write 1 byte
   delay(100000);
   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_DO, 0x00);                       // Power down magnetometer  
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and write 1 byte
   delay(100000);
   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_DO, 0x0F);                       // Enter fuze mode
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and write 1 byte
   delay(100000);
   
   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_ASAX);               // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and read 3 bytes
   delay(100000);
   readBytes(MPUnum, I2C_SLV4_DI, 1, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
   magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;        // Return x-axis sensitivity adjustment values, etc.


   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_ASAY);               // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and read 3 bytes
   delay(100000);
   readBytes(MPUnum, I2C_SLV4_DI, 1, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
   magCalibration[1] =  (float)(rawData[0] - 128)/256.0f + 1.0f;        // Return x-axis sensitivity adjustment values, etc.
 

   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_ASAZ);               // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and read 3 bytes
   delay(100000);
   readBytes(MPUnum, I2C_SLV4_DI, 1, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
   magCalibration[2] =  (float)(rawData[0] - 128)/256.0f + 1.0f;        // Return x-axis sensitivity adjustment values, etc.
 

   //magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
   //magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
   
   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_DO, 0x00);                       // Power down magnetometer  
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and transfer 1 byte
   delay(100000);

   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer 
   // Configure the magnetometer for continuous read and highest resolution
   // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
   // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
   writeByte(MPUnum, I2C_SLV4_DO, Mscale << 4 | Mmode);        // Set magnetometer data resolution and sample ODR
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and transfer 1 byte
   delay(100000);
}





void MPU9250_readMagData(volatile regs_mapping_t* MPUnum, int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
//  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_XOUT_L);             // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and read 7 bytes
   delay(100000);
   readBytes(MPUnum, I2C_SLV4_DI, 1, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
  
   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_XOUT_H);             // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and read 7 bytes
   delay(100000);
   readBytes(MPUnum, I2C_SLV4_DI, 1, &rawData[1]);        // Read the x-, y-, and z-axis calibration values
  
   destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value



   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_YOUT_L);             // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and read 7 bytes
   delay(100000);
   readBytes(MPUnum, I2C_SLV4_DI, 1, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
  
   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_YOUT_H);             // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and read 7 bytes
   delay(100000);
   readBytes(MPUnum, I2C_SLV4_DI, 1, &rawData[1]);        // Read the x-, y-, and z-axis calibration values
  
   destination[1] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value



   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_ZOUT_L);             // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and read 7 bytes
   delay(100000);
   readBytes(MPUnum, I2C_SLV4_DI, 1, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
  
   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_ZOUT_H);             // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and read 7 bytes
   delay(100000);
   readBytes(MPUnum, I2C_SLV4_DI, 1, &rawData[1]);        // Read the x-, y-, and z-axis calibration values
  
   destination[2] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value





   writeByte(MPUnum, I2C_SLV4_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV4_REG, AK8963_ST2);             // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV4_CTRL, 0x80);                     // Enable I2C and read 7 bytes
   delay(100000);
   readBytes(MPUnum, I2C_SLV4_DI, 1, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
  
   uint8_t c = rawData[0];  // Turn the MSB and LSB into a signed 16-bit value

    if((c & 0x08))
        jtaguart_puts("overflow\n");
 /*uint8_t c = rawData[6]; // End data read by reading ST2 register
   if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
   destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
   destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
   destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
   } else {
    jtaguart_puts("overflow\n");
    }*/
}

void MPU9250_readMPU9250Data(volatile regs_mapping_t* MPUnum, int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readBytes(MPUnum, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;   
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;  
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;  
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ; 
}


volatile int* pio_n = (volatile int*)(0x01000000);

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
                       
//    print_int(whoiam,1);
    float   magCalibration1[3] = {0, 0, 0};
    MPU9250_initMPU9250(regs,AFS_2G,GFS_250DPS,0x04);
    delay(100000); 

    uint8_t c = MPU9250_getAK8963CID(regs);

    print_int(c,1);

    MPU9250_initAK8963Slave(regs,MFS_16BITS,M_100Hz,magCalibration1);

    print_float(magCalibration1[0],1);
    print_float(magCalibration1[1],1);
    print_float(magCalibration1[2],1);



    for(;;) {

        //uart_rs232_buffer_tx_process(&tx_state);


        chr = jtaguart_getchar();
        if (chr != 0)
        {
            switch (chr)
            {

                case 'm':
                {
                    MPU9250_initAK8963Slave(regs,MFS_16BITS,M_100Hz,magCalibration1);
                    uint8_t c = MPU9250_getAK8963CID(regs);
                    print_int(c,1);
                    break;
                }

                case 'M':
                {
                    print_int(readByte(regs,EXT_SENS_DATA_00),1);
                    break;
                }

                case 'd':
                {
                    mpu_9250_sensors_data_t data_values;
                    mpu_9250_mag_data_t mag_values;

                    MPU9250_readMPU9250Data(regs,&data_values);
                    MPU9250_readMagData(regs,&mag_values);

                    jtaguart_puts("ACC / TEMP / GYRO\n");
                    for (i=0;i<sizeof(data_values)/2;i++)
                    {
                        print_int(((int16_t*)&data_values)[i],1);
                    }
                    jtaguart_puts("MAG\n");
                    for (i=0;i<sizeof(mag_values)/2;i++)
                    {
                        print_int(((int16_t*)&mag_values)[i],1);
                    }
            
                    break;
                }

                case 'S':
                {
                    spi_read(regs,0x3B,buf,2);
                    print_int(buf[0],1); 
                    print_int(buf[1],1); 
                    break;
                }


                case 's':
                {
                    mpu_9250_sensors_regs_t data_regs;
                    mpu_9250_sensors_data_t data_values;

                    spi_read(regs,0x3B,&data_regs,sizeof(data_regs));

                    jtaguart_puts("raw spi data\n");
                    for (i=0;i<20;i++)
                        print_int(regs->spi_data[i],1);

                    jtaguart_puts("read regs\n");
                    for (i=0;i<sizeof(data_regs);i++)
                        print_int(((uint8_t*)&data_regs.accel_xout_h)[i],1);


                    print_int(data_regs.accel_xout_h,1); 
                    print_int(data_regs.accel_xout_l,1); 

                    for (i=0;i<sizeof(data_regs)/2;i++)
                    {
                        uint8_t *ptr_in = &((uint8_t*)&data_regs)[i*2];
/*
                        jtaguart_puts("in\n");
                        print_int(ptr_in[0],1);
                        print_int(ptr_in[1],1);
                        jtaguart_puts("out\n");
*/

                        uint8_t *ptr_out = &((uint8_t*)&data_values)[i*2];

                        *((int16_t*)ptr_out) =  ((int16_t)ptr_in[1]) << 8 | ptr_in[0];
                        print_int(*((int16_t*)ptr_out),1);
                    }



                    break;
                }


                case 'r':

                    return 0;
                case 'p':
                    jtaguart_puts("----- IMU ----\n");

                    print_int(readByte(regs,0x75),1); 

                    spi_read(regs,0x74,buf,2);
                    print_int(buf[1],1); 

                    spi_read(regs,0x75,&whoiam,1);
                    print_int(whoiam,1); 

                    spi_read(regs,0x75,buf,2);
                    print_int(buf[0],1); 




                    for (i=0;i<20;i++)
                        print_int(regs->spi_data[i],1);

/*
                    jtaguart_puts("----- PID Internals ----\n");
                    if (regs->arg[0] == 0)
                    {
                        print_float(pid.f_target,1);                        
                        print_float(regs->f_measure,1);                        
                    } else {
                        print_int(pid.target,1);                        
                        print_int(regs->measure,1); 
                    }
                    print_float(pid.qr.previous_out,1);                        
                    print_float(pid.qr.previous_var,1);                        
                    print_int(regs->enable,1);
                    print_int(regs->override,1);
                    print_float(regs->speed,1);
                    print_float(regs->acc,1);
                    print_int(regs->sat,1);

                    print_float(pid.error,1);
                    print_float(pid.x_integral,1);
                    print_float(pid.saturated,1);
                    print_float(pid.x,1);
                    print_int(period_latest,1);
*/
                    break;
            }

        }

	}
}

