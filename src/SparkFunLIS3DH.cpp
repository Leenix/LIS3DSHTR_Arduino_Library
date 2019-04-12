/******************************************************************************
SparkFunLIS3DH.cpp
LIS3DSH Arduino and Teensy Driver

Marshall Taylor @ SparkFun Electronics
Nov 16, 2016
https://github.com/sparkfun/LIS3DH_Breakout
https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library

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

*/

#include "SparkFunLIS3DH.h"
#include "SPI.h"
#include "Wire.h"

#pragma region core

/**
 * Configure communication options for the accelerometer.
 * @param bus_type: I2C_MODE or SPI_MODE to indicate the type of comms used.
 * @param address_or_cs: I2C address or chip select pin, depending on comm type used.
 */
LIS3DSH::LIS3DSH(uint8_t bus_type, uint8_t address_or_cs) {
    comm_type = bus_type;
    if (comm_type == I2C_MODE) {
        i2c_address = address_or_cs;
    }

    else if (comm_type == SPI_MODE) {
        chip_select_pin = address_or_cs;
    }

    // Accelerometer settings
    settings.sample_rate = 3;          // Hz.  Can be: 3, 6, 12, 25, 50, 100, 400, 800, 1600
    settings.accelerometer_range = 2;  // Max G force readable.  Can be: 2, 4, 8, 16

    settings.accelerometer_x_enabled = true;
    settings.accelerometer_y_enabled = true;
    settings.accelerometer_z_enabled = true;

    // FIFO control settings
    settings.fifo_enabled = 0;
    settings.fifo_watermark = 20;  // Can be 0 to 32
    settings.fifo_mode = 0;        // FIFO mode.

    allOnesCounter = 0;
    nonSuccessCounter = 0;
}

/**
 * Prepare the device for use.
 * Communications are established and initial settings are applied.
 */
status_t LIS3DSH::begin(void) {
    status_t comm_result = begin_comms();
    apply_settings();

    return comm_result;
}

/**
 * Start up sensor communication.
 * @return: 0 == Success; 1 == Failure to read HW ID
 */
status_t LIS3DSH::begin_comms() {
    status_t comm_result = IMU_SUCCESS;

    if (comm_type == SPI_MODE) {
        pinMode(chip_select_pin, OUTPUT);
        digitalWrite(chip_select_pin, HIGH);
        SPI.begin();

#if defined(ARDUINO_ARCH_ESP32)
        SPI.setFrequency(1000000);
        SPI.setBitOrder(SPI_MSBFIRST);
        // Like the standard arduino/teensy comment below, mode0 seems wrong according to standards
        // but conforms to the timing diagrams when used for the ESP32
        SPI.setDataMode(SPI_MODE0);

#elif defined(__MK20DX256__)

        // Maximum SPI frequency is 10MHz, could divide by 2 here:
        SPI.setClockDivider(SPI_CLOCK_DIV4);
        SPI.setBitOrder(MSBFIRST);
        // Data is captured on rising edge of clock (CPHA = 0)
        // Base value of the clock is HIGH (CPOL = 1)
        // MODE0 for Teensy 3.1 operation
        SPI.setDataMode(SPI_MODE0);
#else
        // probably __AVR__
        // Maximum SPI frequency is 10MHz, could divide by 2 here:
        SPI.setClockDivider(SPI_CLOCK_DIV4);
        SPI.setBitOrder(MSBFIRST);
        // Data is captured on rising edge of clock (CPHA = 0)
        // Base value of the clock is HIGH (CPOL = 1)
        // MODE3 for 328p operation
        SPI.setDataMode(SPI_MODE3);

#endif
    }

    delay(2);

    // Check the ID register to determine if the operation was a success.
    uint8_t read_check;
    read(&read_check, LIS3DSH_WHO_AM_I);
    if (read_check != WHO_AM_I_ID) {
        comm_result = IMU_HW_ERROR;
    }

    return comm_result;
}

/**
 * Write the device configuration to the device.
 * All settings should be configured before calling begin().
 * If any settings are changed, this method should be called again to update the device configuration.
 */
void LIS3DSH::apply_settings(void) {
    set_sample_rate();
    set_range_and_aa();
    configure_interrupts();
    configure_fifo();
}

/**
 * Read a the contents of a single register.
 * @param output: Pointer to the output data location.
 * @param address: Register address to read from.
 * @return: Success/failure of the read operation.
 */
status_t LIS3DSH::read(uint8_t *output, uint8_t address) {
    uint8_t result;
    uint8_t num_bytes_to_read = 1;
    status_t comm_result = IMU_SUCCESS;

    switch (comm_type) {
        case I2C_MODE:
            Wire.beginTransmission(i2c_address);
            Wire.write(address);
            if (Wire.endTransmission() != 0) {
                comm_result = IMU_HW_ERROR;
            }
            Wire.requestFrom(i2c_address, num_bytes_to_read);
            while (Wire.available())  // slave may send less than requested
            {
                result = Wire.read();  // receive a byte as a proper uint8_t
            }
            break;

        case SPI_MODE:
            // take the chip select low to select the device:
            digitalWrite(chip_select_pin, LOW);
            // send the device the register you want to read:
            SPI.transfer(address | 0x80);  // Ored with "read request" bit
            // send a value of 0 to read the first byte returned:
            result = SPI.transfer(0x00);
            // take the chip select high to de-select:
            digitalWrite(chip_select_pin, HIGH);

            if (result == 0xFF) {
                // we've recieved all ones, report
                comm_result = IMU_ALL_ONES_WARNING;
            }
            break;

        default:
            break;
    }

    *output = result;
    return comm_result;
}

/**
 * Read multiple bytes from the device at once.
 * Useful for reading the entire FIFO buffer, among other things.
 * The output buffer must be large enough to contain the number of bytes
 * requested, or an overflow will occur.
 *
 * @param output: Pointer to buffer that will contain the read output.
 * @param address: Starting register to read data from.
 * @param length: Number of bytes to read.
 * @return: Error/success result of read.
 */
status_t LIS3DSH::burst_read(uint8_t *output, uint8_t address, uint8_t length) {
    status_t comm_result = IMU_SUCCESS;

    // define pointer that will point to the external space
    uint8_t i = 0;
    uint8_t c = 0;
    uint8_t tempFFCounter = 0;

    switch (comm_type) {
        case I2C_MODE:
            Wire.beginTransmission(i2c_address);
            address |= 0x80;  // turn auto-increment bit on, bit 7 for I2C
            Wire.write(address);
            if (Wire.endTransmission() != 0) {
                comm_result = IMU_HW_ERROR;
            } else  // OK, all worked, keep going
            {
                Wire.requestFrom(i2c_address, length);
                while ((Wire.available()) && (i < length))  // slave may send less than requested
                {
                    c = Wire.read();  // receive a byte as character
                    *output = c;
                    output++;
                    i++;
                }
            }
            break;

        case SPI_MODE:
            // take the chip select low to select the device:
            digitalWrite(chip_select_pin, LOW);
            // send the device the register you want to read:
            SPI.transfer(address | 0x80 | 0x40);  // Ored with "read request" bit and "auto increment" bit
            while (i < length)                    // slave may send less than requested
            {
                c = SPI.transfer(0x00);  // receive a byte as character
                if (c == 0xFF) {
                    // May have problem
                    tempFFCounter++;
                }
                *output = c;
                output++;
                i++;
            }
            if (tempFFCounter == i) {
                // Ok, we've recieved all ones, report
                comm_result = IMU_ALL_ONES_WARNING;
            }
            // take the chip select high to de-select:
            digitalWrite(chip_select_pin, HIGH);
            break;

        default:
            break;
    }

    return comm_result;
}

/**
 * Read a 16-bit value from the device.
 * The data is read from two sequential registers and combined.
 * Useful for reading accelerometer values, which are spread across two registers.
 *
 * @param output: Pointer to output variable.
 * @param address: Starting address of register to read from.
 * @return: Success/error result of the read.
 */
status_t LIS3DSH::read_16(int16_t *output, uint8_t address) {
    {
        uint8_t myBuffer[2];
        status_t comm_result = burst_read(myBuffer, address, 2);
        int16_t result = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
        *output = result;
        return comm_result;
    }
}

/**
 * Write a value to a register.
 *
 * @param input: Byte to write to the register.
 * @param address: Address of register to write to.
 * @return: Success/error result of the write.
 */
status_t LIS3DSH::write(uint8_t input, uint8_t address) {
    status_t comm_result = IMU_SUCCESS;
    switch (comm_type) {
        case I2C_MODE:
            // Write the byte
            Wire.beginTransmission(i2c_address);
            Wire.write(address);
            Wire.write(input);
            if (Wire.endTransmission() != 0) {
                comm_result = IMU_HW_ERROR;
            }
            break;

        case SPI_MODE:
            // take the chip select low to select the device:
            digitalWrite(chip_select_pin, LOW);
            // send the device the register you want to read:
            SPI.transfer(address);
            // send a value of 0 to read the first byte returned:
            SPI.transfer(input);
            // decrement the number of bytes left to read:
            // take the chip select high to de-select:
            digitalWrite(chip_select_pin, HIGH);
            break;

            // No way to check error on this write (Except to read back but that's not reliable)

        default:
            break;
    }

    return comm_result;
}

/**
 * Write a value to a single bit within a register.
 *
 * @param input: Byte to write to the register.
 * @param address: Address of register to write to.
 * @param bit: Bit address to be written within register.
 * @return: Success/error result of the write.
 */
status_t LIS3DSH::write_bit(uint8_t input, uint8_t address, uint8_t bit) {
    status_t comm_result = IMU_SUCCESS;
    uint8_t current_state;

    // Ensure input is a 1-bit flag
    input &= 0x1;

    comm_result = read(&current_state, address);
    current_state |= (input << bit);
    comm_result = write(current_state, address);

    return comm_result;
}

void LIS3DSH::set_sample_rate() {
    uint8_t input = 0;
    switch (settings.sample_rate) {
        case 3:
            input |= (0x01 << 4);
            break;
        case 6:
            input |= (0x02 << 4);
            break;
        case 12:
            input |= (0x03 << 4);
            break;
        case 25:
            input |= (0x04 << 4);
            break;
        case 50:
            input |= (0x05 << 4);
            break;
        case 100:
            input |= (0x06 << 4);
            break;
        case 200:
            input |= (0x07 << 4);
            break;
        case 400:
            input |= (0x08 << 4);
            break;
        case 1600:
            input |= (0x09 << 4);
            break;
        default:
            input = 0;  // Power down
            break;
    }

    input |= (settings.block_data_update & 0x01) << 3;
    input |= (settings.accelerometer_x_enabled & 0x01) << 2;
    input |= (settings.accelerometer_y_enabled & 0x01) << 1;
    input |= (settings.accelerometer_z_enabled & 0x01);
    write(LIS3DSH_CTRL_REG4, input);
}

void LIS3DSH::set_range_and_aa() {
    uint8_t input = 0;
    uint8_t setting;

    // AA filter bandwidth
    switch (settings.aa_filter_bandwidth) {
        case 50:
            setting = 0b11;
            break;

        case 200:
            setting = 0b01;
            break;

        case 400:
            setting = 0b10;
            break;

        default:
        case 800:
            setting = 0b00;
            break;
    }
    input |= (setting << 6);

    // Range
    switch (settings.accelerometer_range) {
        default:
        case 2:
            setting = 0b000;
            break;
        case 4:
            setting = 0b001;
            break;
        case 6:
            setting = 0b010;
            break;
        case 8:
            setting = 0b011;
            break;
        case 16:
            setting = 0b100;
            break;
    }
    input |= (setting << 3);

    // Now, write the patched together data
    write(LIS3DSH_CTRL_REG5, input);
}

void LIS3DSH::configure_fifo() {
    // Constrain some settings
    settings.fifo_enabled = bool(settings.fifo_enabled);
    settings.fifo_watermark_interrupt_enabled = bool(settings.fifo_watermark_interrupt_enabled);
    uint8_t fifo_mode = constrain(settings.fifo_mode, 0, 0b111);
    uint8_t watermark = constrain(settings.fifo_watermark, 0, 32);

    // Enable/disable fifo - CTRL_REG_6
    uint8_t register_value = 0;
    bitSet(register_value, 4);  // Enabled ADD_INC - automatic address increment during burst read
    bitWrite(register_value, 6, settings.fifo_enabled);
    bitWrite(register_value, 2, settings.fifo_watermark_interrupt_enabled);
    write(register_value, LIS3DSH_CTRL_REG6);

    // Set up FIFO options - FIFO_CTRL
    register_value = (fifo_mode << 5) | watermark;
    write(register_value, LIS3DSH_FIFO_CTRL);
}

void LIS3DSH::configure_interrupts() {
    uint8_t interrupt_level = bool(settings.interrupt_polarity);
    uint8_t interrupt_latch = bool(settings.interrupt_latching);
    uint8_t interrupt_2 = bool(settings.interrupt_2_enabled);

    uint8_t register_value = 0;
    bitWrite(register_value, 6, interrupt_level);
    bitWrite(register_value, 5, interrupt_latch);
    bitWrite(register_value, 4, interrupt_2);

    if (settings.fifo_watermark_interrupt_enabled) {
        bitSet(register_value, 3);
    }
    write(register_value, LIS3DSH_CTRL_REG3);
}

#pragma endregion core

///////////////////////////////////////////////////////////////////////////////
// Device methods

int8_t LIS3DSH::read_temperature() {
    uint8_t temperature;
    read(&temperature, LIS3DSH_OUT_T);
    temperature += 25;
    return temperature;
}

float LIS3DSH::calculate_acceleration_from_raw(int16_t input) {
    float output;
    uint8_t range = constrain(settings.accelerometer_range, 2, 16);
    float divisor = 0x7FFF / settings.accelerometer_range;
    return float(output / divisor);
}

/**
 * Read in the contents of the FIFO buffer.
 * A maximum of 32 measurements can be read in at once.
 * Oldest measurements are collected first.
 * Ensure the specified output buffer has enough space to fit the entire buffer (max. 192 bytes).
 *
 * @param output_buffer: Output buffer to store the collected data.
 * @return: The number of measurements read from the FIFO buffer.
 */
uint8_t LIS3DSH::read_fifo_buffer(uint8_t *output_buffer) {
    uint8_t entries_to_read = get_fifo_count();
    if (entries_to_read > 0) {
        uint8_t bytes_to_read = constrain(entries_to_read * 6, 0, 192);
        burst_read(output_buffer, LIS3DSH_X_L, bytes_to_read);
    }

    return entries_to_read;
}

uint8_t LIS3DSH::read_fifo_buffer(AccelerometerEntry *buffer) {
    uint8_t data[192];
    uint8_t num_entries = read_fifo_buffer(data);

    for (size_t i = 0; i < num_entries; i++) {
        buffer[i].x = uint16_t(data[i * 6]) | (data[i * 6 + 1] << 8);
        buffer[i].y = uint16_t(data[i * 6 + 2]) | (data[i * 6 + 3] << 8);
        buffer[i].z = uint16_t(data[i * 6 + 4]) | (data[i * 6 + 5] << 8);
    }
}

uint8_t LIS3DSH::get_fifo_count() {
    uint8_t fifo_state;
    read(&fifo_state, LIS3DSH_FIFO_SRC);
    uint8_t entries_in_fifo = (fifo_state & 0b11111);
    return entries_in_fifo;
}

uint8_t LIS3DSH::has_fifo_overrun() {
    uint8_t fifo_state;
    read(&fifo_state, LIS3DSH_FIFO_SRC);
    return bitRead(fifo_state, 6);
}

void LIS3DSH::power_down() { write(0, LIS3DSH_CTRL_REG4); }

void LIS3DSH::measurement_mode() { set_sample_rate(); }

void LIS3DSH::read_accelerometers(uint16_t *readings) {
    uint8_t data[6];
    burst_read(data, LIS3DSH_X_L, 6);

    for (size_t i = 0; i < 3; i++) {
        readings[i] = (data[i * 2] | (data[i * 2 + 1] << 8));
    }
}