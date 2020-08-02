/*
 */
#ifndef _MPU9250_REGISTER_MAP_H_
#define _MPU9250_REGISTER_MAP_H_

const uint8_t MPU9250_BIT_RESET = 0x80;

const uint8_t MPU9250_SELF_TEST_X_GYRO =  0x00;
const uint8_t MPU9250_SELF_TEST_Y_GYRO =  0x01;
const uint8_t MPU9250_SELF_TEST_Z_GYRO =  0x02;
const uint8_t MPU9250_SELF_TEST_X_ACCEL = 0x0D;
const uint8_t MPU9250_SELF_TEST_Y_ACCEL = 0x0E;
const uint8_t MPU9250_SELF_TEST_Z_ACCEL = 0x0F;
const uint8_t MPU9250_XG_OFFSET_H =       0x13;
const uint8_t MPU9250_XG_OFFSET_L =       0x14;
const uint8_t MPU9250_YG_OFFSET_H =       0x15;
const uint8_t MPU9250_YG_OFFSET_L =       0x16;
const uint8_t MPU9250_ZG_OFFSET_H =       0x17;
const uint8_t MPU9250_ZG_OFFSET_L =       0x18;
const uint8_t MPU9250_SMPLRT_DIV =        0x19;
const uint8_t MPU9250_CONFIG =            0x1A;
const uint8_t MPU9250_GYRO_CONFIG =       0x1B;
const uint8_t MPU9250_ACCEL_CONFIG =      0x1C;
const uint8_t MPU9250_ACCEL_CONFIG_2 =    0x1D;
const uint8_t MPU9250_LP_ACCEL_ODR =      0x1E;
const uint8_t MPU9250_WOM_THR =           0x1F;
const uint8_t MPU9250_FIFO_EN =           0x23;
const uint8_t MPU9250_I2C_MST_CTRL =      0x24;
const uint8_t MPU9250_I2C_SLV0_ADDR =     0x25;
const uint8_t MPU9250_I2C_SLV0_REG =      0x26;
const uint8_t MPU9250_I2C_SLV0_CTRL =     0x27;
const uint8_t MPU9250_I2C_SLV1_ADDR =     0x28;
const uint8_t MPU9250_I2C_SLV1_REG =      0x29;
const uint8_t MPU9250_I2C_SLV1_CTRL =     0x2A;
const uint8_t MPU9250_I2C_SLV2_ADDR =     0x2B;
const uint8_t MPU9250_I2C_SLV2_REG =      0x2C;
const uint8_t MPU9250_I2C_SLV2_CTRL =     0x2D;
const uint8_t MPU9250_I2C_SLV3_ADDR =     0x2E;
const uint8_t MPU9250_I2C_SLV3_REG =      0x2F;
const uint8_t MPU9250_I2C_SLV3_CTRL =     0x30;
const uint8_t MPU9250_I2C_SLV4_ADDR =     0x31;
const uint8_t MPU9250_I2C_SLV4_REG =      0x32;
const uint8_t MPU9250_I2C_SLV4_DO =       0x33;
const uint8_t MPU9250_I2C_SLV4_CTRL =     0x34;
const uint8_t MPU9250_I2C_SLV4_DI =       0x35;
const uint8_t MPU9250_I2C_MST_STATUS =    0x36;
const uint8_t MPU9250_INT_PIN_CFG =       0x37;
const uint8_t MPU9250_INT_ENABLE =        0x38;
const uint8_t MPU9250_INT_STATUS =        0x3A;
const uint8_t MPU9250_ACCEL_XOUT_H =      0x3B;
const uint8_t MPU9250_ACCEL_XOUT_L =      0x3C;
const uint8_t MPU9250_ACCEL_YOUT_H =      0x3D;
const uint8_t MPU9250_ACCEL_YOUT_L =      0x3E;
const uint8_t MPU9250_ACCEL_ZOUT_H =      0x3F;
const uint8_t MPU9250_ACCEL_ZOUT_L =      0x40;
const uint8_t MPU9250_TEMP_OUT_H =        0x41;
const uint8_t MPU9250_TEMP_OUT_L =        0x42;
const uint8_t MPU9250_GYRO_XOUT_H =       0x43;
const uint8_t MPU9250_GYRO_XOUT_L =       0x44;
const uint8_t MPU9250_GYRO_YOUT_H =       0x45;
const uint8_t MPU9250_GYRO_YOUT_L =       0x46;
const uint8_t MPU9250_GYRO_ZOUT_H =       0x47;
const uint8_t MPU9250_GYRO_ZOUT_L =       0x48;
const uint8_t MPU9250_EXT_SENS_DATA_00 =  0x49;
const uint8_t MPU9250_EXT_SENS_DATA_01 =  0x4A;
const uint8_t MPU9250_EXT_SENS_DATA_02 =  0x4B;
const uint8_t MPU9250_EXT_SENS_DATA_03 =  0x4C;
const uint8_t MPU9250_EXT_SENS_DATA_04 =  0x4D;
const uint8_t MPU9250_EXT_SENS_DATA_05 =  0x4E;
const uint8_t MPU9250_EXT_SENS_DATA_06 =  0x4F;
const uint8_t MPU9250_EXT_SENS_DATA_07 =  0x50;
const uint8_t MPU9250_EXT_SENS_DATA_08 =  0x51;
const uint8_t MPU9250_EXT_SENS_DATA_09 =  0x52;
const uint8_t MPU9250_EXT_SENS_DATA_10 =  0x53;
const uint8_t MPU9250_EXT_SENS_DATA_11 =  0x54;
const uint8_t MPU9250_EXT_SENS_DATA_12 =  0x55;
const uint8_t MPU9250_EXT_SENS_DATA_13 =  0x56;
const uint8_t MPU9250_EXT_SENS_DATA_14 =  0x57;
const uint8_t MPU9250_EXT_SENS_DATA_15 =  0x58;
const uint8_t MPU9250_EXT_SENS_DATA_16 =  0x59;
const uint8_t MPU9250_EXT_SENS_DATA_17 =  0x5A;
const uint8_t MPU9250_EXT_SENS_DATA_18 =  0x5B;
const uint8_t MPU9250_EXT_SENS_DATA_19 =  0x5C;
const uint8_t MPU9250_EXT_SENS_DATA_20 =  0x5D;
const uint8_t MPU9250_EXT_SENS_DATA_21 =  0x5E;
const uint8_t MPU9250_EXT_SENS_DATA_22 =  0x5F;
const uint8_t MPU9250_EXT_SENS_DATA_23 =  0x60;
const uint8_t MPU9250_I2C_SLV0_DO =       0x63;
const uint8_t MPU9250_I2C_SLV1_DO =       0x64;
const uint8_t MPU9250_I2C_SLV2_DO =       0x65;
const uint8_t MPU9250_I2C_SLV3_DO =       0x66;
const uint8_t MPU9250_I2C_MST_DELAY_CTRL =0x67;
const uint8_t MPU9250_SIGNAL_PATH_RESET = 0x68;
const uint8_t MPU9250_MOT_DETECT_CTRL =   0x69;
const uint8_t MPU9250_USER_CTRL =         0x6A;
const uint8_t MPU9250_PWR_MGMT_1 =        0x6B;
const uint8_t MPU9250_PWR_MGMT_2 =        0x6C;
const uint8_t MPU9250_FIFO_COUNTH =       0x72;
const uint8_t MPU9250_FIFO_COUNTL =       0x73;
const uint8_t MPU9250_FIFO_R_W =          0x74;
const uint8_t MPU9250_WHO_AM_I =          0x75;
const uint8_t MPU9250_XA_OFFSET_H =       0x77;
const uint8_t MPU9250_XA_OFFSET_L =       0x78;
const uint8_t MPU9250_YA_OFFSET_H =       0x7A;
const uint8_t MPU9250_YA_OFFSET_L =       0x7B;
const uint8_t MPU9250_ZA_OFFSET_H =       0x7D;
const uint8_t MPU9250_ZA_OFFSET_L =       0x7E;

enum interrupt_status_bits {
	INT_STATUS_RAW_DATA_RDY_INT = 0,
	INT_STATUS_FSYNC_INT = 3,
	INT_STATUS_FIFO_OVERFLOW_INT = 4,
	INT_STATUS_WOM_INT = 6,
};

enum gyro_config_bits {
	GYRO_CONFIG_FCHOICE_B = 0,
	GYRO_CONFIG_GYRO_FS_SEL = 3,
	GYRO_CONFIG_ZGYRO_CTEN = 5,
	GYRO_CONFIG_YGYRO_CTEN = 6,
	GYRO_CONFIG_XGYRO_CTEN = 7,
};
#define MPU9250_GYRO_FS_SEL_MASK 0x3
#define MPU9250_GYRO_FCHOICE_MASK 0x3

enum accel_config_bit {
	ACCEL_CONFIG_ACCEL_FS_SEL = 3,
	ACCEL_CONFIG_AZ_ST_EN = 5,
	ACCEL_CONFIG_AY_ST_EN = 6,
	ACCEL_CONFIG_AX_ST_EN = 7,
};
#define MPU9250_ACCEL_FS_SEL_MASK 0x3

enum accel_config_2_bits {
	ACCEL_CONFIG_2_A_DLPFCFG = 0,
	ACCEL_CONFIG_2_ACCEL_FCHOICE_B = 3,
};
	
enum pwr_mgmt_1_bits {
	PWR_MGMT_1_CLKSEL = 0,
	PWR_MGMT_1_PD_PTAT = 3,
	PWR_MGMT_1_GYRO_STANDBY = 4,
	PWR_MGMT_1_CYCLE = 5,
	PWR_MGMT_1_SLEEP = 6,
	PWR_MGMT_1_H_RESET = 7
};

enum pwr_mgmt_2_bits {
	PWR_MGMT_2_DISABLE_ZG = 0,
	PWR_MGMT_2_DISABLE_YG = 1,
	PWR_MGMT_2_DISABLE_XG = 2,
	PWR_MGMT_2_DISABLE_ZA = 3,
	PWR_MGMT_2_DISABLE_YA = 4,
	PWR_MGMT_2_DISABLE_XA = 5,
};

enum int_enable_bits {
	INT_ENABLE_RAW_RDY_EN = 0,
	INT_ENABLE_FSYNC_INT_EN = 3,
	INT_ENABLE_FIFO_OVERFLOW_EN = 4,
	INT_ENABLE_WOM_EN = 6,
};

enum int_pin_cfg_bits {
	INT_PIN_CFG_BYPASS_EN = 1,
	INT_PIN_CFG_FSYNC_INT_MODE_EN = 2,
	INT_PIN_CFG_ACTL_FSYNC = 3,
	INT_PIN_CFG_INT_ANYRD_2CLEAR = 4,
	INT_PIN_CFG_LATCH_INT_EN = 5,
	INT_PIN_CFG_OPEN = 6,
	INT_PIN_CFG_ACTL = 7,
};
#define INT_PIN_CFG_INT_MASK 0xF0

#define MPU9250_WHO_AM_I_RESULT 0x71

const uint8_t AK8963_WHO_AM_I = 0x0;
const uint8_t AK8963_INFO = 0x1;
const uint8_t AK8963_ST1 = 0x2;
const uint8_t AK8963_XOUT_L = 0x3;
const uint8_t AK8963_XOUT_H = 0x4;
const uint8_t AK8963_YOUT_L = 0x5;
const uint8_t AK8963_YOUT_H = 0x6;
const uint8_t AK8963_ZOUT_L = 0x7;
const uint8_t AK8963_ZOUT_H = 0x8;
const uint8_t AK8963_ST2 = 0x9;
const uint8_t AK8963_CNTL = 0xA;
const uint8_t AK8963_RSV = 0xB;
const uint8_t AK8963_ASTC = 0xC;
const uint8_t AK8963_TS1 = 0xD;
const uint8_t AK8963_TS2 = 0xE;
const uint8_t AK8963_I2CDIS = 0xF;
const uint8_t AK8963_ASAX = 0x10;
const uint8_t AK8963_ASAY = 0x11;
const uint8_t AK8963_ASAZ = 0x12;

#define MAG_CTRL_OP_MODE_MASK 0xF

#define AK8963_ST1_DRDY_BIT 0

#define AK8963_WHO_AM_I_RESULT 0x48

#endif // _MPU9250_REGISTER_MAP_H_
