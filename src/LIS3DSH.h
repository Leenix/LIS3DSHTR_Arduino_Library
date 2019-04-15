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
#include <Arduino.h>

#ifndef LIS3DSH_H
#define LIS3DSH_H

#define I2C_MODE 0
#define SPI_MODE 1

const uint8_t DEFAULT_I2C_ADDRESS = 0x3C;
const uint8_t WHO_AM_I_ID = 0b111111;

typedef enum RETURN_CODES
{
    IMU_SUCCESS = 0,
    IMU_HW_ERROR = 1,
    IMU_NOT_SUPPORTED = 2,
    IMU_GENERIC_ERROR = 3,
    IMU_OUT_OF_BOUNDS = 4,
    IMU_ALL_ONES_WARNING = 5,
} status_t;

enum LIS3DSH_FIFO_MODES
{
    LIS3DSH_FIFO_BYPASS = 0b000,
    LIS3DSH_FIFO_MODE = 0b001,
    LIS3DSH_FIFO_STREAM = 0b010,
    LIS3DSH_FIFO_STREAM_TO_FIFO = 0b011,
    LIS3DSH_FIFO_BYPASS_TO_STREAM = 0b100,
    LIS3DSH_FIFO_BYPASS_TO_FIFO = 0b111,
};

enum LIS3DSH_INTERRUPT_POLARITY
{
    LIS3DSH_ACTIVE_LOW = 0,
    LIS3DSH_ACTIVE_HIGH = 1,

};

enum
{
    LIS3DSH_INTERRUPT_LATCHED = 0,
    LIS3DSH_INTERRUPT_PULSED = 1,
};

enum STATE_MACHINES
{
    SM1 = 0,
    SM2 = 1,
};

typedef struct SensorSettings
{
    // Accelerometer settings
    uint8_t accelerometer_range = 2; // Max G force readable. [2, 4, 8, 16]

    uint16_t aa_filter_bandwidth = 800; // Hz. [50, 200, 400, 800]
    uint16_t sample_rate = 0;           // Hz. [0 (off), 3, 6, 12, 25, 50, 100, 400, 800, 1600]
    uint8_t accelerometer_x_enabled = true;
    uint8_t accelerometer_y_enabled = true;
    uint8_t accelerometer_z_enabled = true;

    // Fifo settings
    uint8_t fifo_enabled = false;
    uint8_t fifo_mode = LIS3DSH_FIFO_BYPASS;
    uint8_t fifo_watermark = 0;
    uint8_t fifo_watermark_interrupt_enabled = false;

    // Interrupt settings
    uint8_t interrupt_polarity = LIS3DSH_ACTIVE_LOW;
    uint8_t interrupt_latching = LIS3DSH_INTERRUPT_LATCHED;
    uint8_t interrupt_1_enabled = false;
    uint8_t interrupt_2_enabled = false;
    uint8_t data_ready_interrupt_enabled;

    uint8_t block_data_update = false;

    uint8_t state_machine_1_enabled = false;
    uint8_t state_machine_2_enabled = false;
};

struct StateMachineSettings
{
    uint8_t hysteresis;
    uint8_t interrupt_output;

    uint8_t code[16];
    uint16_t timer1_initial_value;
    uint16_t timer2_initial_value;
    uint8_t timer3_initial_value;
    uint8_t timer4_initial_value;
    uint8_t threshold_1;
    uint8_t threshold_2;
    uint8_t mask_a;
    uint8_t mask_b;
    uint8_t thresholds_are_absolute;
    uint8_t stop_and_cont_interrupts;

    // SM2 Only
    uint8_t decimator = 0;
    uint8_t diff_calculation_enabled;
    uint8_t diff_from_constants_enabled;
};

struct AccelerometerEntry
{
    int16_t x;
    int16_t y;
    int16_t z;
};

///////////////////////////////////////////////////////////////////////////////
// LIS3DSH Class

class LIS3DSH
{
public:
    LIS3DSH(uint8_t comm_mode = I2C_MODE, uint8_t address_or_cs = DEFAULT_I2C_ADDRESS);
    status_t begin(void);
    status_t read(uint8_t *output, uint8_t address);
    status_t burst_read(uint8_t *output, uint8_t address, uint8_t length);
    status_t read_16(int16_t *output, uint8_t address);
    status_t write(uint8_t input, uint8_t address);
    status_t burst_write(uint8_t *output, uint8_t address, uint8_t length);
    status_t write_bit(uint8_t input, uint8_t address, uint8_t bit);

    void apply_settings();
    void power_down();
    void measurement_mode();

    uint8_t read_fifo_buffer(uint8_t *buffer);
    uint8_t read_fifo_buffer(AccelerometerEntry *buffer);
    uint8_t get_fifo_count();
    uint8_t has_fifo_overrun();

    int8_t read_temperature(void);
    void read_accelerometers(uint16_t *readings);
    void read_accelerometers(AccelerometerEntry *entry);
    float calculate_acceleration_from_raw(int16_t);

    void set_state_machine_instruction(uint8_t sm_number, uint8_t code_register_id, uint8_t instruction);
    void set_state_machine_instruction(uint8_t sm_number, uint8_t code_register_id, uint8_t reset_instruction,
                                       uint8_t next_instruction);
    void write_state_machine_status(uint8_t sm_number, uint8_t active_status);

    void configure_auto_sleep();
    void configure_state_machines();

    // Error checking
    SensorSettings settings;
    StateMachineSettings sm_settings[2];
    uint16_t allOnesCounter;
    uint16_t nonSuccessCounter;

private:
    status_t begin_comms();
    void set_sample_rate();
    void set_range_and_aa();
    void configure_fifo();
    void configure_interrupts();
    void configure_state_machine(uint8_t sm_number);

    // Communication stuff
    uint8_t comm_type;
    uint8_t i2c_address;
    uint8_t chip_select_pin;
    uint8_t state_machine_1_configured = false;
    uint8_t state_machine_2_configured = false;
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
#define LIS3DSH_TIM2_1 0x52 // 16-bit, Low byte first
#define LIS3DSH_TIM1_1 0x54 // 16-bit, Low byte first
#define LIS3DSH_THRS2_1 0x56
#define LIS3DSH_THRS1_1 0x57
#define LIS3DSH_MASK1_B 0x59
#define LIS3DSH_MASK1_A 0x5A
#define LIS3DSH_SETT1 0x5B
#define LIS3DSH_PR1 0x5C
#define LIS3DSH_TC1 0x5D   // 16-bit, Low byte first
#define LIS3DSH_OUTS1 0x5F // 16-bit, Low byte first

// State Machine 2 registers
#define LIS3DSH_ST2_1 0x60 // ST2_1 (60) ~ ST2_16 (6F)
#define LIS3DSH_TIM4_2 0x70
#define LIS3DSH_TIM3_2 0x71
#define LIS3DSH_TIM2_2 0x72 // 16-bit, Low byte first
#define LIS3DSH_TIM1_2 0x74 // 16-bit, Low byte first
#define LIS3DSH_THRS2_2 0x76
#define LIS3DSH_THRS1_2 0x77
#define LIS3DSH_DES2 0x78
#define LIS3DSH_MASK2_B 0x79
#define LIS3DSH_MASK2_A 0x7A
#define LIS3DSH_SETT2 0x7B
#define LIS3DSH_PR2 0x7C
#define LIS3DSH_TC2 0x7D   // 16-bit, Low byte first
#define LIS3DSH_OUTS2 0x7F // 16-bit, Low byte first

///////////////////////////////////////////////////////////////////////////////
// State Machine opcodes and commands

// Operations
#define LIS3DSH_NOP 0x0
#define LIS3DSH_TI1 0x1
#define LIS3DSH_TI2 0x2
#define LIS3DSH_TI3 0x3
#define LIS3DSH_TI4 0x4
#define LIS3DSH_GNTH1 0x5
#define LIS3DSH_GNTH2 0x6
#define LIS3DSH_LNTH1 0x7
#define LIS3DSH_LNTH2 0x8
#define LIS3DSH_GTTH1 0x9
#define LIS3DSH_LLTH2 0xA
#define LIS3DSH_GRTH1 0xB
#define LIS3DSH_LRTH1 0xC
#define LIS3DSH_GRTH2 0xD
#define LIS3DSH_LRTH2 0xE
#define LIS3DSH_NZERO 0xF

// Commands

#define LIS3DSH_STOP 0x00
#define LIS3DSH_CONT 0x11
#define LIS3DSH_JMP 0x22
#define LIS3DSH_SRP 0x33
#define LIS3DSH_CRP 0x44
#define LIS3DSH_SETP 0x55
#define LIS3DSH_SETS1 0x66
#define LIS3DSH_STHR1 0x77
#define LIS3DSH_OUTC 0x88
#define LIS3DSH_OUTW 0x99
#define LIS3DSH_STHR2 0xAA
#define LIS3DSH_DEC 0xBB
#define LIS3DSH_SISW 0xCC
#define LIS3DSH_REL 0xDD
#define LIS3DSH_STHR3 0xEE
#define LIS3DSH_SSYNC 0xFF
#define LIS3DSH_SABS0 0x12
#define LIS3DSH_SABS1 0x13
#define LIS3DSH_SELMA 0x14
#define LIS3DSH_SRADI0 0x21
#define LIS3DSH_SRADI1 0x23
#define LIS3DSH_SELSA 0x24
#define LIS3DSH_SCS0 0x31
#define LIS3DSH_SCS1 0x32
#define LIS3DSH_SRTAM0 0x34
#define LIS3DSH_STIM3 0x41
#define LIS3DSH_STIM4 0x42
#define LIS3DSH_SRTAM1 0x43

#endif
