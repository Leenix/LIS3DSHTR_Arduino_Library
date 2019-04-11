/******************************************************************************
 * TODO - Redo documentation
 * 
SparkFunLIS3DSH.h
LIS3DSH Arduino and Teensy Driver

Marshall Taylor @ SparkFun Electronics
Nov 16, 2016
https://github.com/sparkfun/LIS3DSH_Breakout
https://github.com/sparkfun/SparkFun_LIS3DSH_Arduino_Library

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __LIS3DSH_IMU_H__
#define __LIS3DSH_IMU_H__

#include "stdint.h"

//values for commInterface
#define I2C_MODE 0
#define SPI_MODE 1

// Return values
typedef enum
{
	IMU_SUCCESS,
	IMU_HW_ERROR,
	IMU_NOT_SUPPORTED,
	IMU_GENERIC_ERROR,
	IMU_OUT_OF_BOUNDS,
	IMU_ALL_ONES_WARNING,
	//...
} status_t;

enum LIS3DSH_FIFO_MODES
{
	LIS3DSH_FIFO_DISABLED = 1,
	LIS3DSH_FIFO_MODE = 2,
	LIS3DSH_FIFO_STREAM = 3,
	LIS3DSH_FIFO_TRIGGER = 4,
}

//This is the core operational class of the driver.
//  LIS3SDHCore contains only read and write operations towards the IMU.
//  To use the higher level functions, use the class LIS3DSH which inherits
//  this class.

class LIS3DSHCore
{
  public:
	LIS3SDHCore(uint8_t);
	LIS3SDHCore(uint8_t, uint8_t);
	~LIS3SDHCore() = default;

	status_t beginCore(void);

	//The following utilities read and write to the IMU

	//ReadRegisterRegion takes a uint8 array address as input and reads
	//  a chunk of memory into that array.
	status_t readRegisterRegion(uint8_t *, uint8_t, uint8_t);

	//readRegister reads one 8-bit register
	status_t readRegister(uint8_t *, uint8_t);

	//Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
	//  Acts as a 16-bit read operation
	status_t readRegisterInt16(int16_t *, uint8_t offset);

	//Writes an 8-bit byte;
	status_t writeRegister(uint8_t, uint8_t);

  private:
	//Communication stuff
	uint8_t commInterface;
	uint8_t I2CAddress;
	uint8_t chipSelectPin;
};

//This struct holds the settings the driver uses to do calculations
struct SensorSettings
{
  public:
	//Accelerometer settings
	uint16_t sample_rate;		 //Hz.  Can be: 0,1,10,25,50,100,200,400,1600 Hz
	uint8_t accelerometer_range; //Max G force readable.  Can be: 2, 4, 8, 16

	uint8_t accelerometer_x_enabled = true;
	uint8_t accelerometer_y_enabled = true;
	uint8_t accelerometer_z_enabled = true;

	//Fifo settings
	uint8_t fifo_enabled;
	uint8_t fifo_mode;
	uint8_t fifo_watermark;

	uint8_t interrupt_polarity;
	uint8_t interrupt_latching;

	uint8_t inactivity_sleep_enabled;
	uint8_t inactivity_threshold;
	uint8_t inactivity_time;
	uint8_t wake_threshold;
};

//This is the highest level class of the driver.
//
//  class LIS3DSH inherits the core and makes use of the beginCore()
//method through it's own begin() method.  It also contains the
//settings struct to hold user settings.

class LIS3DSH : public LIS3SDHCore
{
  public:
	SensorSettings settings;

	//Error checking
	uint16_t allOnesCounter;
	uint16_t nonSuccessCounter;

	LIS3DSH(uint8_t busType = I2C_MODE, uint8_t inputArg = 0x19);

	status_t begin(void);
	void apply_settings(void);

	void power_down(void);

	int8_t read_temperature(void);
	void read_accelerometers(uint16_t *readings);

	//Returns the raw bits from the sensor cast as 16-bit signed integers
	int16_t readRawAccelX(void);
	int16_t readRawAccelY(void);
	int16_t readRawAccelZ(void);

	//Returns the values as floats.  Inside, this calls readRaw___();
	float readFloatAccelX(void);
	float readFloatAccelY(void);
	float readFloatAccelZ(void);

	//FIFO stuff
	uint8_t read_fifo(uint8_t *buffer);
	uint8_t get_fifo_count();

	float calcAccel(int16_t);

  private:
};

///////////////////////////////////////////////////////////////////////////////
// Device Registers

// Temperature output
#define LIS3DSH_OUT_T 0x0C

// Information registers
#define LIS3DSH_INFO1 0x0D
#define LIS3DSH_INFO2 0x0E

// Who am I ID
#define LIS3DSH_WHO_AM_I 0x0F

// Offset correction
#define LIS3DSH_OFF_X 0x10
#define LIS3DSH_OFF_Y 0x11
#define LIS3DSH_OFF_Z 0x12

// Constant shift
#define LIS3DSH_CS_X 0x13
#define LIS3DSH_CS_Y 0x14
#define LIS3DSH_CS_Z 0x15

// Long counter registers
#define LIS3DSH_LC_L 0x16
#define LIS3DSH_LC_H 0x17

// Interrupt status
#define LIS3DSH_STAT 0x18

// Peak values
#define LIS3DSH_PEAK1 0x19
#define LIS3DSH_PEAK2 0x1A

// Vector filter coefficient
#define LIS3DSH_VFC_1 0x1B
#define LIS3DSH_VFC_2 0x1C
#define LIS3DSH_VFC_3 0x1D
#define LIS3DSH_VFC_4 0x1E

// Threshold value 3
#define LIS3DSH_THRS3 0x1F

// Control registers
#define LIS3DSH_CTRL_REG1 0x21
#define LIS3DSH_CTRL_REG2 0x22
#define LIS3DSH_CTRL_REG3 0x23
#define LIS3DSH_CTRL_REG4 0x20
#define LIS3DSH_CTRL_REG5 0x24
#define LIS3DSH_CTRL_REG6 0x25

// Status data register
#define LIS3DSH_STATUS 0x27

// Output registers
#define LIS3DSH_X_L 0x28
#define LIS3DSH_X_H 0x29
#define LIS3DSH_Y_L 0x2A
#define LIS3DSH_Y_H 0x2B
#define LIS3DSH_Z_L 0x2C
#define LIS3DSH_Z_H 0x2D

// FIFO registers
#define LIS3DSH_FIFO_CTRL 0x2E
#define LIS3DSH_FIFO_SRC 0x2F

// State Machine 1 registers
#define LIS3DSH_ST1_1 0x40
#define LIS3DSH_TIM4_1 0x50
#define LIS3DSH_TIM3_1 0x51
#define LIS3DSH_TIM2_1 0x52 //16-bit, Low byte first
#define LIS3DSH_TIM1_1 0x54 //16-bit, Low byte first
#define LIS3DSH_THRS2_1 0x56
#define LIS3DSH_THRS1_1 0x57
#define LIS3DSH_MASK1_B 0x59
#define LIS3DSH_MASK1_A 0x5A
#define LIS3DSH_SETT1 0x5B
#define LIS3DSH_PR1 0x5C
#define LIS3DSH_TC1 0x5D   //16-bit, Low byte first
#define LIS3DSH_OUTS1 0x5F //16-bit, Low byte first

// State Machine 2 registers
#define LIS3DSH_ST2_1 0x60 // ST2_1 (60) ~ ST2_16 (6F)
#define LIS3DSH_TIM4_2 0x70
#define LIS3DSH_TIM3_2 0x71
#define LIS3DSH_TIM2_2 0x72 //16-bit, Low byte first
#define LIS3DSH_TIM1_2 0x74 //16-bit, Low byte first
#define LIS3DSH_THRS2_2 0x76
#define LIS3DSH_THRS1_2 0x77
#define LIS3DSH_DES2 0x78
#define LIS3DSH_MASK1_2 0x79
#define LIS3DSH_MASK1_2 0x7A
#define LIS3DSH_SETT2 0x7B
#define LIS3DSH_PR2 0x7C
#define LIS3DSH_TC2 0x7D   //16-bit, Low byte first
#define LIS3DSH_OUTS2 0x7F //16-bit, Low byte first

#endif
