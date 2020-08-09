// #define USE_WIRE
//#define DEBUG

#ifdef USB_SERIAL_HID
#define DEBUG
#endif

#include "MPU9250_RegisterMap.h"


#ifdef USE_WIRE
#include <Wire.h>
#else
#ifdef __IMXRT1062__
#include <i2c_driver.h>
#include "imx_rt1060/imx_rt1060_i2c_driver.h"
#endif
#endif

#ifdef __IMXRT1062__
#include "usb_dev.h"
#endif

#define PIN_GREEN       2
#define PIN_RED         3
#define PIN_YELLOW      4
#define PIN_BLUE        5
#define PIN_ORANGE      6
#define PIN_STRUMUP     7
#define PIN_STRUMDOWN   8
#define PIN_PEDAL       9
#define PIN_DPADLED    10
#define PIN_SELECT     11
#define PIN_START      12
#define PIN_WHAMMY     14
#define PIN_MPU9250SCL 16
#define PIN_MPU9250SDA 17
#define PIN_GH5NECKSDA 18
#define PIN_GH5NECKSCL 19
#define PIN_DPADR      20
#define PIN_DPADL      21
#define PIN_DPADD      22
#define PIN_DPADU      23

#define WHAMMY_DEADZONE 0x2
#define WHAMMY_MAX 0xFE

#ifndef USE_WIRE
I2CMaster& GH5Neck = Master;
I2CMaster& MPU9250 = Master1;

const uint16_t gh5neck_addr = 0x0D;
const uint16_t mpu9250_addr = 0x68;
const uint16_t ak8963_addr = 0x0C;
const uint8_t MPU9250_ADDRESS = 0x68;
const uint8_t AK8963_ADDRESS = 0x0C;

const uint8_t gh5neck_reg_slidernew = 0x15;

uint8_t mpu9250_readbuffer[32] = {};
uint8_t mpu9250_writebuffer[8] = {};

uint8_t gh5neck_curslider = 0;
#endif

volatile uint16_t current_buttons = 0x3FFF;
volatile uint8_t current_whammy = 0x00;
elapsedMicros sinceSend = 0;
int32_t pedalCyclesOn = 0;
int32_t bootloaderCounter = -1;




#ifdef USB_XINPUT_GUITAR
#include "usb_xinput_guitar.h"

uint8_t xinput_tx[20];
uint8_t xinput_tx_prev[20];
uint8_t xinput_rx[8];
uint8_t xinput_rx_prev[8];
#endif


#ifdef USE_WIRE
uint8_t GH5Neck_I2C_OK() {
  Wire.beginTransmission(0x0D);
  Wire.write(0x11);
  Wire.endTransmission();
  //delayMicroseconds(100);
  Wire.requestFrom(0x0D, 1);
  while (true)
  {
    if ( Wire.available() )
      break;
  }
  return Wire.read();
}

uint8_t GH5Neck_I2C_Buttons() {
  Wire.beginTransmission(0x0D);
  Wire.write(0x12);
  Wire.endTransmission();
  //delayMicroseconds(100);
  Wire.requestFrom(0x0D, 1);
  while (true)
  {
    if ( Wire.available() )
      break;
  }
  return Wire.read();
}

// Extended GH5 slider with full multi-touch
uint8_t GH5Neck_I2C_SliderNew() {
  Wire.beginTransmission(0x0D);
  Wire.write(0x15);
  Wire.endTransmission();
  //delayMicroseconds(100);
  Wire.requestFrom(0x0D, 1);
  while (true)
  {
    if ( Wire.available() )
      break;
  }
  return Wire.read();
}

// Older style slider with WT-type detection, adjacent frets only
uint8_t GH5Neck_I2C_SliderOld() {
  Wire.beginTransmission(0x0D);
  Wire.write(0x16);
  Wire.endTransmission();
  //delayMicroseconds(100);
  Wire.requestFrom(0x0D, 1);
  while (true)
  {
    if ( Wire.available() )
      break;
  }
  return Wire.read();
}

uint8_t MPU9250_WhoAmI() {
  Wire1.beginTransmission(0x68);
  Wire1.write(0x75);
  Wire1.endTransmission();
  //delayMicroseconds(100);
  Wire1.requestFrom(0x68, 1);
  while (true)
  {
    if ( Wire1.available() )
      break;
  }
  return Wire1.read();
}

#else

// TODO: Move hardware I2C stuff into here, actually make both methods work...

#endif


void UpdateButtons() {
  uint16_t ret = 0;
  ret |= (digitalRead(PIN_GREEN));
  ret |= (digitalRead(PIN_RED) << 1);
  ret |= (digitalRead(PIN_YELLOW) << 2);
  ret |= (digitalRead(PIN_BLUE) << 3);
  ret |= (digitalRead(PIN_ORANGE) << 4);
  ret |= (digitalRead(PIN_STRUMUP) << 5);
  ret |= (digitalRead(PIN_STRUMDOWN) << 6);
  ret |= (digitalRead(PIN_SELECT) << 7);
  ret |= (digitalRead(PIN_START) << 8);
  ret |= (digitalRead(PIN_PEDAL) << 9);
  ret |= (digitalRead(PIN_DPADL) << 10);
  ret |= (digitalRead(PIN_DPADR) << 11);
  ret |= (digitalRead(PIN_DPADU) << 12);
  ret |= (digitalRead(PIN_DPADD) << 13);
  current_buttons = ret;
}

void UpdateWhammy() {
  uint8_t ret = 0;
  ret = analogRead(PIN_WHAMMY);
  if ( ret <= WHAMMY_DEADZONE )
    ret = 0;
  else if ( ret >= WHAMMY_MAX )
    ret = 255;

  current_whammy = ret;
}

void mpu9250_i2c_finish() {
  elapsedMillis timeout;
  while (timeout < 200) {
    if (MPU9250.finished()) {
      return;
    }
  }
#ifdef DEBUG
  Serial.println("MPU9250: ERROR timed out waiting for transfer to finish.");
#endif
}

void gh5neck_i2c_finish() {
  elapsedMillis timeout;
  while (timeout < 200) {
    if (GH5Neck.finished()) {
      return;
    }
  }
#ifdef DEBUG
  Serial.println("GH5Neck: ERROR timed out waiting for transfer to finish.");
#endif
}

void UpdateSlider() {
  if ( GH5Neck.finished() ) {
    GH5Neck.write_async(gh5neck_addr, (uint8_t*)&gh5neck_reg_slidernew, sizeof(gh5neck_reg_slidernew), false);
    gh5neck_i2c_finish();
    GH5Neck.read_async(gh5neck_addr, &gh5neck_curslider, sizeof(gh5neck_curslider), true);
    // don't finish here, we count on using the 0.5ms this takes to do other crap and account for the delay accordingly
    //gh5neck_i2c_finish();
  }
}



// I2C read/write functions for the MPU9250 and AK8963 sensors

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  memset(mpu9250_writebuffer, 0, sizeof(mpu9250_writebuffer));
  mpu9250_writebuffer[0] = subAddress;
  mpu9250_writebuffer[1] = data;
  MPU9250.write_async(address, (uint8_t*)&mpu9250_writebuffer, 2, true);
  mpu9250_i2c_finish();
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  memset(mpu9250_writebuffer, 0, sizeof(mpu9250_writebuffer));
  memset(mpu9250_readbuffer, 0, sizeof(mpu9250_readbuffer));

  mpu9250_writebuffer[0] = subAddress;
  MPU9250.write_async(address, (uint8_t*)&mpu9250_writebuffer, 1, false);
  mpu9250_i2c_finish();

  MPU9250.read_async(address, (uint8_t*)&mpu9250_readbuffer, 1, true);
  mpu9250_i2c_finish();

  return mpu9250_readbuffer[0];
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
  memset(mpu9250_writebuffer, 0, sizeof(mpu9250_writebuffer));
  memset(mpu9250_readbuffer, 0, sizeof(mpu9250_readbuffer));

  mpu9250_writebuffer[0] = subAddress;
  MPU9250.write_async(address, (uint8_t*)&mpu9250_writebuffer, 1, false);
  mpu9250_i2c_finish();

  MPU9250.read_async(address, (uint8_t*)&mpu9250_readbuffer, count, true);
  mpu9250_i2c_finish();
  memcpy(dest, mpu9250_readbuffer, count);
}

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature
float SelfTest[6];            // holds results of gyro and accelerometer self test

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)

bool newMagData = false;

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

  writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, FS << 3); // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG_2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

  for (int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer
    readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for (int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer
    readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.; // Report percent differences
    destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
  }
}

void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}

void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
      break;
  }
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, MPU9250_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, MPU9250_FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  /*  writeByte(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, data[0]);
    writeByte(MPU9250_ADDRESS, MPU9250_XA_OFFSET_L, data[1]);
    writeByte(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, data[2]);
    writeByte(MPU9250_ADDRESS, MPU9250_YA_OFFSET_L, data[3]);
    writeByte(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, data[4]);
    writeByte(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_L, data[5]);
  */
  // Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

void initMPU9250()
{
  // wake up device
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG_2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG_2, c); // Write new ACCEL_CONFIG2 register value

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  //   writeByte(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear
  writeByte(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}

void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  newMagData = (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
  if (newMagData == true) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
  }
}

void magcalMPU9250(float * dest1, float * dest2)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = { -32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

#ifdef DEBUG
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
#endif
  delay(4000);

  // shoot for ~fifteen seconds of mag data
  if (Mmode == 0x02) sample_count = 128; // at 8 Hz ODR, new mag data is available every 125 ms
  if (Mmode == 0x06) sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms
  for (ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if (Mmode == 0x02) delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
    if (Mmode == 0x06) delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
  }

  //    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  //    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  //    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * mRes * magCalibration[0]; // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * mRes * magCalibration[1];
  dest1[2] = (float) mag_bias[2] * mRes * magCalibration[2];

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  dest2[0] = avg_rad / ((float)mag_scale[0]);
  dest2[1] = avg_rad / ((float)mag_scale[1]);
  dest2[2] = avg_rad / ((float)mag_scale[2]);

#ifdef DEBUG
  Serial.println("Mag Calibration done!");
#endif
}

void readMPU9250Data(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}














void setup() {
  // put your setup code here, to run once:
#ifdef DEBUG
  Serial.begin(250000);
#endif
#ifdef USE_WIRE
  Wire.setSDA(PIN_GH5NECKSDA);
  Wire.setSCL(PIN_GH5NECKSCL);
  Wire.setClock(100000);
  Wire.begin();

  Wire1.setSDA(PIN_MPU9250SDA);
  Wire1.setSCL(PIN_MPU9250SCL);
  Wire1.setClock(400000);
  Wire1.begin();
#else
  GH5Neck.begin(100000);
  MPU9250.begin(400000);
#endif

  pinMode(PIN_DPADLED, OUTPUT);
  digitalWrite(PIN_DPADLED, HIGH);

  pinMode(PIN_DPADLED,   OUTPUT);
  pinMode(PIN_GREEN,     INPUT_PULLUP);
  pinMode(PIN_RED,       INPUT_PULLUP);
  pinMode(PIN_YELLOW,    INPUT_PULLUP);
  pinMode(PIN_BLUE,      INPUT_PULLUP);
  pinMode(PIN_ORANGE,    INPUT_PULLUP);
  pinMode(PIN_STRUMUP,   INPUT_PULLUP);
  pinMode(PIN_STRUMDOWN, INPUT_PULLUP);
  pinMode(PIN_PEDAL,     INPUT_PULLUP);
  pinMode(PIN_START,     INPUT_PULLUP);
  pinMode(PIN_SELECT,    INPUT_PULLUP);
  pinMode(PIN_DPADU,     INPUT_PULLUP);
  pinMode(PIN_DPADD,     INPUT_PULLUP);
  pinMode(PIN_DPADL,     INPUT_PULLUP);
  pinMode(PIN_DPADR,     INPUT_PULLUP);
  pinMode(PIN_WHAMMY,    INPUT);

  analogReadRes(8);
  analogReadAveraging(1);

#if defined(USB_HID) || defined(USB_SERIAL_HID)
  Joystick.useManualSend(true);
#endif

  // MPU9250 setup
  uint8_t whoami = readByte(mpu9250_addr, MPU9250_WHO_AM_I);
#ifdef DEBUG
  Serial.print( "MPU9250 Who Am I?: 0x" );
  Serial.println(whoami, HEX);
#endif
  MPU9250SelfTest(SelfTest);
#ifdef DEBUG
  Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
  Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
  Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");
#endif
  delay(1000);
  // get sensor resolutions, only need to do this once
  getAres();
  getGres();
  getMres();

  accelgyrocalMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
#ifdef DEBUG
  Serial.println(" Calibrate gyro and accel");
  Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
  Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
#endif

  delay(1000);

  initMPU9250();
#ifdef DEBUG
  Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
#endif
  uint8_t d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
#ifdef DEBUG
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
#endif

  initAK8963(magCalibration);
#ifdef DEBUG
  Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
#endif

  // TO-DO: EEPROM
  magBias[0] = 450;
  magBias[1] = 400;
  magBias[2] = -150;
  magScale[0] = 1.0;
  magScale[1] = 1.0;
  magScale[2] = 1.0;

  //magcalMPU9250(magBias, magScale);

#ifdef DEBUG
  Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
  Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]);
  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
#endif

  pedalCyclesOn = 0;
  bootloaderCounter = -1;
  // zero out sinceSend due to lengthy startup now
  sinceSend = 0;

  // don't check useless crap in yield
  yield_active_check_flags = 0;

#ifdef USB_XINPUT_GUITAR
  // preset constants in tx
  xinput_tx[0] = 0x00;
  xinput_tx[1] = 0x14;
#endif
}




void loop() {
  UpdateSlider();
  readMPU9250Data(MPU9250Data);
  ax = (float)MPU9250Data[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
  ay = (float)MPU9250Data[1] * aRes - accelBias[1];
  az = (float)MPU9250Data[2] * aRes - accelBias[2];
  gx = (float)MPU9250Data[4] * gRes; // get actual gyro value, this depends on scale being set
  gy = (float)MPU9250Data[5] * gRes;
  gz = (float)MPU9250Data[6] * gRes;
  UpdateWhammy();
  UpdateButtons();

  while ( sinceSend < 900 ) {
    UpdateButtons();
  }

  // our dpad will actually allow all buttons to be pressed.
  // we can use that for programming purposes, but for now,
  // lets just prioritize up and right.
  bool up = !(current_buttons & 0b1000000000000);
  bool down = !(current_buttons & 0b10000000000000);
  bool left = !(current_buttons & 0b10000000000);
  bool right = !(current_buttons & 0b100000000000);

  bool green = !(current_buttons & 0b1);
  bool red = !(current_buttons & 0b10);
  bool yellow = !(current_buttons & 0b100);
  bool blue = !(current_buttons & 0b1000);
  bool orange = !(current_buttons & 0b10000);
  bool slidergreen = (gh5neck_curslider & 0b10000);
  bool sliderred = (gh5neck_curslider & 0b1000);
  bool slideryellow = (gh5neck_curslider & 0b100);
  bool sliderblue = (gh5neck_curslider & 0b10);
  bool sliderorange = (gh5neck_curslider & 0b1);

  bool strumup = !(current_buttons & 0b100000);
  bool strumdown = !(current_buttons & 0b1000000);
  bool start = !(current_buttons & 0b100000000);
  bool select = !(current_buttons & 0b10000000);
  bool pedal = !(current_buttons & 0b1000000000);

  // gyro activate select as well
  select = select || gx >= 200 || gy >= 200 || gz >= 200 || gx <= -200 || gy <= -200 || gz <= -200;

  // pedal activate select as well but do not hold

  if ( pedalCyclesOn < 0 ) {
    pedal = true;
    select = true;
    pedalCyclesOn++;
  } else if ( pedal ) {
    if ( pedalCyclesOn < 1000 ) {
      pedalCyclesOn++;
      select = true;
    }
  } else {
    // activate on long release as well
    if ( pedalCyclesOn == 1000 ) {
      select = true;
      // Hold pedal/select for 20ms. Discovered during testing on GH3 instead of CH
      pedalCyclesOn = -20;
    }
  }

#if defined(USB_XINPUT_GUITAR)
  bool newXinputData = true;
  uint8_t button1 = (up || strumup) | ((down || strumdown) << 1) |
                    (left << 2) | (right << 3) |
                    (start << 4) | (select << 5);
  uint8_t button2 = (orange || sliderorange) |
                    (pedal << 1) |
                    ((green || slidergreen) << 4) |
                    ((red || sliderred) << 5) |
                    ((blue || sliderblue) << 6) |
                    ((yellow || slideryellow) << 7);

  xinput_tx[2] = button1;
  xinput_tx[3] = button2;

  int16_t out_whammy = map(current_whammy, 0, 255, -32768, 32767);
  xinput_tx[10] = (uint8_t)((out_whammy) & 0xFF);
  xinput_tx[11] = (uint8_t)((out_whammy >> 8) & 0xFF);

  if( !memcmp( xinput_tx, xinput_tx_prev, sizeof(xinput_tx) ) )
    newXinputData = false;
  else
    memcpy( xinput_tx_prev, xinput_tx, sizeof(xinput_tx) );
#elif defined(USB_HID) || defined(USB_SERIAL_HID)
  int16_t dpadangle = -1;
  if ( up ) {
    if ( right ) {
      dpadangle = 45;
    } else if ( left ) {
      dpadangle = 315;
    } else {
      dpadangle = 0;
    }
  } else if ( down ) {
    if ( right ) {
      dpadangle = 135;
    } else if ( left ) {
      dpadangle = 225;
    } else {
      dpadangle = 180;
    }
  } else if ( right ) {
    dpadangle = 90;
  } else if ( left ) {
    dpadangle = 270;
  }
  Joystick.hat( dpadangle );
  Joystick.button( 1, green || slidergreen ? 1 : 0 );
  Joystick.button( 2, red || sliderred ? 1 : 0 );
  Joystick.button( 3, yellow || slideryellow ? 1 : 0 );
  Joystick.button( 4, blue || sliderblue ? 1 : 0 );
  Joystick.button( 5, orange || sliderorange ? 1 : 0 );
  Joystick.button( 6, strumup ? 1 : 0 );
  Joystick.button( 7, strumdown ? 1 : 0 );
  Joystick.button( 8, select ? 1 : 0 );
  Joystick.button( 9, start ? 1 : 0 );
  Joystick.button( 10, pedal ? 1 : 0 );

  int16_t whammyToJX = 512;
  whammyToJX += (current_whammy * 2);
  if( whammyToJX == 1022 ) // 512 + 255 * 2
    whammyToJX = 1023;

  Joystick.X( whammyToJX );

  int16_t acceleroToJY = 512;
  acceleroToJY += (ax * 512);

  if ( acceleroToJY > 1023 ) {
    acceleroToJY = 1023;
  } else if ( acceleroToJY < 0 ) {
    acceleroToJY = 0;
  }

  Joystick.Y( acceleroToJY );
#endif

  if ( bootloaderCounter > 5000 ) {
    bootloaderCounter++;
    goto skipSend;
  }

  if ( green && red && yellow && blue && orange && start && select && strumup ) {
    bootloaderCounter++;
    goto skipSend;
  } else {
    bootloaderCounter = -1;
  }

  while ( sinceSend <= 1000 ) {
    ;
  }
#if defined(USB_XINPUT_GUITAR)
  if( newXinputData ) {
    XInputGuitar.send( xinput_tx, 10 );
  }
  XInputGuitar.recv( xinput_rx, 0 );
#elif defined(USB_HID) || defined(USB_SERIAL_HID)
  Joystick.send_now();
#endif
  sinceSend -= 1000;

#ifdef DEBUG
  /*
    Serial.println( "ax, ay, az, gx, gy, gz: " );
    Serial.println( ax, 3 );
    Serial.println( ay, 3 );
    Serial.println( az, 3 );
    Serial.println( gx, 3 );
    Serial.println( gy, 3 );
    Serial.println( gz, 3 );*/
  //Serial.println(sinceSend, DEC);
#endif
  return;

skipSend:
  sinceSend = 0;
  if ( bootloaderCounter >= 5000 ) {
    if ( bootloaderCounter >= 9000 ) {
      digitalWrite(PIN_DPADLED, LOW);
    } else if ( bootloaderCounter >= 8000 ) {
      digitalWrite(PIN_DPADLED, HIGH);
    } else if ( bootloaderCounter >= 7000 ) {
      digitalWrite(PIN_DPADLED, LOW);
    } else if ( bootloaderCounter >= 6000 ) {
      digitalWrite(PIN_DPADLED, HIGH);
    } else {
      digitalWrite(PIN_DPADLED, LOW);
    }
#ifdef __IMXRT1062__
    if ( bootloaderCounter == 5000 ) {
      usb_start_sof_interrupts(NUM_INTERFACE);
    } else if ( bootloaderCounter >= 10000 ) {
      usb_stop_sof_interrupts(NUM_INTERFACE);
      asm("bkpt #251");
    }
#elif (defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__))
    __asm__ volatile("bkpt");
#endif
  }
}
