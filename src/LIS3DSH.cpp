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

#include "LIS3DSH.h"
#include "SPI.h"
#include "Wire.h"

/**
 * Configure communication options for the accelerometer.
 * @param bus_type: I2C_MODE or SPI_MODE to indicate the type of comms used.
 * @param address_or_cs: I2C address or chip select pin, depending on comm type used.
 */
LIS3DSH::LIS3DSH(uint8_t bus_type, uint8_t address_or_cs)
{
    if (bus_type == SPI_MODE)
    {
        comm_type = SPI_MODE;
        chip_select_pin = address_or_cs;
    }

    else
    {
        comm_type = I2C_MODE;
        i2c_address = address_or_cs;
    }
}

/**
 * Prepare the device for use.
 * Communications are established and initial settings are applied.
 */
status_t LIS3DSH::begin(void)
{
    status_t comm_result = begin_comms();
    apply_settings();

    return comm_result;
}

/**
 * Start up sensor communication.
 * @return: 0 == Success; 1 == Failure to read_from HW ID
 */
status_t LIS3DSH::begin_comms()
{
    if (comm_type == SPI_MODE)
    {
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
    return comms_check();
}

status_t LIS3DSH::comms_check()
{
    uint8_t read_check;
    status_t comm_result = IMU_HW_ERROR;

    read_from(&read_check, LIS3DSH_WHO_AM_I);
    if (read_check == WHO_AM_I_ID)
    {
        comm_result = IMU_SUCCESS;
    }

    return comm_result;
}

/**
 * Write the device configuration to the device.
 * All settings should be configured before calling begin().
 * If any settings are changed, this method should be called again to update the device configuration.
 */
void LIS3DSH::apply_settings(void)
{
    set_sample_rate();
    set_range_and_aa();
    configure_interrupts();
    configure_fifo();
    configure_state_machines();
}

/**
 * Read a the contents of a single register.
 * @param output: Pointer to the output data location.
 * @param address: Register address to read_from from.
 * @return: Success/failure of the read_from operation.
 */
status_t LIS3DSH::read_from(uint8_t *output, uint8_t address)
{
    uint8_t result;
    uint8_t num_bytes_to_read = 1;
    status_t comm_result = IMU_SUCCESS;

    switch (comm_type)
    {
    case I2C_MODE:
        Wire.beginTransmission(i2c_address);
        Wire.write(address);
        Wire.endTransmission();

        Wire.requestFrom(i2c_address, num_bytes_to_read);
        while (!Wire.available()) // slave may send less than requested
        {
            result = Wire.read_from(); // receive a byte as a proper uint8_t
        }
        break;

    case SPI_MODE:
        // take the chip select low to select the device:
        digitalWrite(chip_select_pin, LOW);
        // send the device the register you want to read_from:
        SPI.transfer(address | 0x80); // Ored with "read_from request" bit
        // send a value of 0 to read_from the first byte returned:
        result = SPI.transfer(0x00);
        // take the chip select high to de-select:
        digitalWrite(chip_select_pin, HIGH);

        if (result == 0xFF)
        {
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
 * @param output: Pointer to buffer that will contain the read_from output.
 * @param address: Starting register to read_from data from.
 * @param length: Number of bytes to read_from.
 * @return: Error/success result of read_from.
 */
status_t LIS3DSH::burst_read(uint8_t *output, uint8_t address, uint8_t length)
{
    status_t comm_result = IMU_SUCCESS;

    // define pointer that will point to the external space
    uint8_t i = 0;
    uint8_t c = 0;
    uint8_t tempFFCounter = 0;

    switch (comm_type)
    {
    case I2C_MODE:
        Wire.beginTransmission(i2c_address);
        address |= 0x80; // turn auto-increment bit on, bit 7 for I2C
        Wire.write(address);
        if (Wire.endTransmission() != 0)
        {
            comm_result = IMU_HW_ERROR;
        }

        else // OK, all worked, keep going
        {
            Wire.requestFrom(i2c_address, length);
            while ((Wire.available()) && (i < length)) // slave may send less than requested
            {
                c = Wire.read_from(); // receive a byte as character
                *output = c;
                output++;
                i++;
            }
        }
        break;

    case SPI_MODE:
        // take the chip select low to select the device:
        digitalWrite(chip_select_pin, LOW);
        // send the device the register you want to read_from:
        SPI.transfer(address | 0x80 | 0x40); // Ored with "read_from request" bit and "auto increment" bit
        while (i < length)                   // slave may send less than requested
        {
            c = SPI.transfer(0x00); // receive a byte as character
            if (c == 0xFF)
            {
                // May have problem
                tempFFCounter++;
            }
            *output = c;
            output++;
            i++;
        }
        if (tempFFCounter == i)
        {
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
 * The data is read_from from two sequential registers and combined.
 * Useful for reading accelerometer values, which are spread across two registers.
 *
 * @param output: Pointer to output variable.
 * @param address: Starting address of register to read_from from.
 * @return: Success/error result of the read_from.
 */
status_t LIS3DSH::read_16(int16_t *output, uint8_t address)
{
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
status_t LIS3DSH::write(uint8_t input, uint8_t address)
{
    status_t comm_result = IMU_SUCCESS;
    switch (comm_type)
    {
    case I2C_MODE:
        // Write the byte
        Wire.beginTransmission(i2c_address);
        Wire.write(address);
        Wire.write(input);
        if (Wire.endTransmission() != 0)
        {
            comm_result = IMU_HW_ERROR;
        }
        break;

    case SPI_MODE:
        // take the chip select low to select the device:
        digitalWrite(chip_select_pin, LOW);
        // send the device the register you want to read_from:
        SPI.transfer(address);
        // send a value of 0 to read_from the first byte returned:
        SPI.transfer(input);
        // decrement the number of bytes left to read_from:
        // take the chip select high to de-select:
        digitalWrite(chip_select_pin, HIGH);
        break;

        // No way to check error on this write (Except to read_from back but that's not reliable)

    default:
        break;
    }

    return comm_result;
}

/**
 * Write a value to a register.
 *
 * @param input: Byte to write to the register.
 * @param address: Address of register to write to.
 * @return: Success/error result of the write.
 */
status_t LIS3DSH::burst_write(uint8_t *input, uint8_t address, uint8_t length)
{
    status_t comm_result = IMU_SUCCESS;
    switch (comm_type)
    {
    case I2C_MODE:
        // Write the bytes
        Wire.beginTransmission(i2c_address);
        Wire.write(address);
        for (size_t i = 0; i < length; i++)
        {
            Wire.write(input[i]);
        }
        if (Wire.endTransmission() != 0)
        {
            comm_result = IMU_HW_ERROR;
        }
        break;

    case SPI_MODE:
        // take the chip select low to select the device:
        digitalWrite(chip_select_pin, LOW);
        // send the device the register you want to read_from:
        SPI.transfer(address);
        // send a value of 0 to read_from the first byte returned:
        for (size_t i = 0; i < length; i++)
        {
            SPI.transfer(input[i]);
        }
        // decrement the number of bytes left to read_from:
        // take the chip select high to de-select:
        digitalWrite(chip_select_pin, HIGH);
        break;

        // No way to check error on this write (Except to read_from back but that's not reliable)

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
status_t LIS3DSH::write_bit(uint8_t input, uint8_t address, uint8_t bit)
{
    status_t comm_result = IMU_SUCCESS;
    uint8_t current_state;

    // Ensure input is a 1-bit flag
    input &= 0x1;

    comm_result = read_from(&current_state, address);
    current_state |= (input << bit);
    comm_result = write(current_state, address);

    return comm_result;
}

void LIS3DSH::set_sample_rate()
{
    uint8_t input = 0;
    switch (settings.sample_rate)
    {
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
        input = 0; // Power down
        break;
    }

    input |= (settings.block_data_update & 0x01) << 3;
    input |= (settings.accelerometer_x_enabled & 0x01) << 2;
    input |= (settings.accelerometer_y_enabled & 0x01) << 1;
    input |= (settings.accelerometer_z_enabled & 0x01);
    write(input, LIS3DSH_CTRL_REG4);
}

void LIS3DSH::set_range_and_aa()
{
    uint8_t input = 0;
    uint8_t setting;

    // AA filter bandwidth
    switch (settings.aa_filter_bandwidth)
    {
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
    switch (settings.accelerometer_range)
    {
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

void LIS3DSH::configure_fifo()
{
    // Constrain some settings
    settings.fifo_enabled = bool(settings.fifo_enabled);
    settings.fifo_watermark_interrupt_enabled = bool(settings.fifo_watermark_interrupt_enabled);
    uint8_t fifo_mode = constrain(settings.fifo_mode, 0, 0b111);
    uint8_t watermark = constrain(settings.fifo_watermark, 0, 32);

    // Enable/disable fifo - CTRL_REG_6
    uint8_t register_value = 0;
    bitSet(register_value, 4); // Enabled ADD_INC - automatic address increment during burst read_from
    bitWrite(register_value, 6, settings.fifo_enabled);
    bitWrite(register_value, 2, settings.fifo_watermark_interrupt_enabled);
    write(register_value, LIS3DSH_CTRL_REG6);

    // Set up FIFO options - FIFO_CTRL
    register_value = (fifo_mode << 5) | watermark;
    write(register_value, LIS3DSH_FIFO_CTRL);
}

void LIS3DSH::configure_interrupts()
{
    uint8_t interrupt_level = bool(settings.interrupt_polarity);
    uint8_t interrupt_latch = bool(settings.interrupt_latching);
    uint8_t interrupt_1 = bool(settings.interrupt_1_enabled);
    uint8_t interrupt_2 = bool(settings.interrupt_2_enabled);
    uint8_t ready_interrupt = bool(settings.data_ready_interrupt_enabled);

    uint8_t register_value = 0;
    bitWrite(register_value, 7, ready_interrupt);
    bitWrite(register_value, 6, interrupt_level);
    bitWrite(register_value, 5, interrupt_latch);
    bitWrite(register_value, 4, interrupt_2);
    bitWrite(register_value, 3, interrupt_1);

    if (settings.fifo_watermark_interrupt_enabled)
    {
        bitSet(register_value, 3);
    }
    write(register_value, LIS3DSH_CTRL_REG3);
}

void LIS3DSH::configure_state_machines()
{
    if (settings.state_machine_1_enabled)
    {
        configure_state_machine(SM1);
        state_machine_1_configured = true;
    }

    if (settings.state_machine_2_enabled)
    {
        configure_state_machine(SM2);
        state_machine_2_configured = true;
    }
}

/**
 * Apply settings for the specified state machine.
 *
 * @param sm_number: State machine to be configured.
 */
void LIS3DSH::configure_state_machine(uint8_t sm_number)
{
    uint8_t offset = 0;
    if (sm_number == SM2)
        offset = 0x20;

    // CTRL_REG[1,2]
    uint8_t register_value = 0;
    register_value |= constrain(sm_settings[sm_number].hysteresis, 0, 7) << 5;
    register_value |= constrain(sm_settings[sm_number].interrupt_output, 0, 1) << 3;
    write(register_value, LIS3DSH_CTRL_REG1 + offset);

    // SM code registers
    for (size_t i = 0; i < 16; i++)
    {
        uint8_t code = sm_settings[sm_number].code[i];
        if (code != 0x00)
        {
            set_state_machine_instruction(sm_number, i + 1, code);
        }
        else
        {
            break;
        }
    }

    // Thresholds
    write(sm_settings[sm_number].threshold_1, LIS3DSH_THRS1_1 + offset);
    write(sm_settings[sm_number].threshold_2, LIS3DSH_THRS2_1 + offset);

    // Timers
    write(lowByte(sm_settings[sm_number].timer1_initial_value), LIS3DSH_TIM1_1 + offset);
    write(highByte(sm_settings[sm_number].timer1_initial_value), LIS3DSH_TIM1_1 + offset + 1);
    write(lowByte(sm_settings[sm_number].timer2_initial_value), LIS3DSH_TIM2_1 + offset);
    write(highByte(sm_settings[sm_number].timer2_initial_value), LIS3DSH_TIM2_1 + offset + 1);
    write(sm_settings[sm_number].timer3_initial_value, LIS3DSH_TIM3_1 + offset);
    write(sm_settings[sm_number].timer4_initial_value, LIS3DSH_TIM4_1 + offset);

    // SM SETT
    register_value = 0;
    register_value |= ((sm_settings[sm_number].thresholds_are_absolute & 0x1) << 5);
    if (sm_number == SM2)
    {
        register_value |= ((sm_settings[SM2].diff_calculation_enabled & 0x1) << 4);
        register_value |= ((sm_settings[SM2].diff_from_constants_enabled & 0x1) << 3);
    }
    register_value |= (sm_settings[sm_number].stop_and_cont_interrupts & 0x1);
    write(register_value, LIS3DSH_SETT1 + offset);

    // Masks
    write(sm_settings[sm_number].mask_a, LIS3DSH_MASK1_A + offset);
    write(sm_settings[sm_number].mask_b, LIS3DSH_MASK1_B + offset);

    // SM2 Only
    if (sm_number == SM2)
    {
        write(sm_settings[sm_number].decimator, LIS3DSH_DES2);
    }

    write_state_machine_status(sm_number, true);
}

///////////////////////////////////////////////////////////////////////////////
// Device methods

int8_t LIS3DSH::read_temperature()
{
    uint8_t temperature;
    read_from(&temperature, LIS3DSH_OUT_T);
    temperature += 25;
    return temperature;
}

float LIS3DSH::calculate_acceleration_from_raw(int16_t input)
{
    float output;
    uint8_t range = constrain(settings.accelerometer_range, 2, 16);
    float divisor = 0x7FFF / settings.accelerometer_range;
    return float(output / divisor);
}

/**
 * Read in the contents of the FIFO buffer.
 * A maximum of 32 measurements can be read_from in at once.
 * Oldest measurements are collected first.
 * Ensure the specified output buffer has enough space to fit the entire buffer (max. 192 bytes).
 *
 * @param output_buffer: Output buffer to store the collected data.
 * @return: The number of measurements read_from from the FIFO buffer.
 */
uint8_t LIS3DSH::read_fifo_buffer(uint8_t *output_buffer)
{
    uint8_t entries_to_read = get_fifo_count();
    if (entries_to_read > 0)
    {
        uint8_t bytes_to_read = constrain(entries_to_read * 6, 0, 192);
        burst_read(output_buffer, LIS3DSH_X_L, bytes_to_read);
    }

    return entries_to_read;
}

uint8_t LIS3DSH::read_fifo_buffer(AccelerometerEntry *buffer)
{
    uint8_t data[192];
    uint8_t num_entries = read_fifo_buffer(data);

    for (size_t i = 0; i < num_entries; i++)
    {
        buffer[i].x = data[i * 6] + (data[i * 6 + 1] << 8);
        buffer[i].y = data[i * 6 + 2] + (data[i * 6 + 3] << 8);
        buffer[i].z = data[i * 6 + 4] + (data[i * 6 + 5] << 8);
    }
    return num_entries;
}

uint8_t LIS3DSH::get_fifo_count()
{
    uint8_t fifo_state;
    read_from(&fifo_state, LIS3DSH_FIFO_SRC);
    uint8_t entries_in_fifo = (fifo_state & 0b11111);
    return entries_in_fifo;
}

uint8_t LIS3DSH::has_fifo_overrun()
{
    uint8_t fifo_state;
    read_from(&fifo_state, LIS3DSH_FIFO_SRC);
    return bitRead(fifo_state, 6);
}

void LIS3DSH::power_down() { write(0, LIS3DSH_CTRL_REG4); }

void LIS3DSH::measurement_mode() { set_sample_rate(); }

void LIS3DSH::read_accelerometers(uint16_t *readings)
{
    uint8_t data[6];
    burst_read(data, LIS3DSH_X_L, 6);

    for (size_t i = 0; i < 3; i++)
    {
        readings[i] = (data[i * 2] | (data[i * 2 + 1] << 8));
    }
}

void LIS3DSH::read_accelerometers(AccelerometerEntry *entry)
{
    uint16_t data[3];
    read_accelerometers(data);
    entry->x = data[0];
    entry->y = data[1];
    entry->z = data[2];
}

void LIS3DSH::set_state_machine_instruction(uint8_t sm_number, uint8_t code_register_id, uint8_t instruction)
{
    // Make sure the inputs are in range
    sm_number = constrain(sm_number, SM1, SM2);
    code_register_id = constrain(code_register_id, 1, 16);

    // Calculate the correct register address
    uint8_t register_address = LIS3DSH_ST1_1;
    if (sm_number == 2)
    {
        register_address = LIS3DSH_ST2_1;
    }
    register_address += (code_register_id - 1);

    write(instruction, register_address);
}

void LIS3DSH::set_state_machine_instruction(uint8_t sm_number, uint8_t code_register_id, uint8_t reset_instruction,
                                            uint8_t next_instruction)
{
    reset_instruction &= 0x0F;
    next_instruction &= 0x0F;
    uint8_t instruction = (reset_instruction << 4) + next_instruction;
    set_state_machine_instruction(sm_number, code_register_id, instruction);
}

void LIS3DSH::write_state_machine_status(uint8_t sm_number, uint8_t active_status)
{
    uint8_t register_address = LIS3DSH_CTRL_REG1;
    if (sm_number == SM2)
    {
        register_address = LIS3DSH_CTRL_REG2;
    }
    write_bit(active_status, register_address, 0);
}

void LIS3DSH::configure_auto_sleep()
{
    settings.state_machine_2_enabled = true;

    StateMachineSettings sleep_settings;

    sleep_settings.threshold_1 = 4; // Wake threshold
    sleep_settings.threshold_2 = 2; // Sleep threshold

    sleep_settings.mask_a = 0b11111100;
    sleep_settings.mask_b = 0b11111100;

    /**
     * The auto-sleep routine is comprised of 3 main states: inactive, active, and pre-sleep.
     * The device starts off in an inactive state.
     * Data is not pushed to FIFO in this state if the Bypass-to-FIFO or Bypass-to-Stream modes are used.
     *
     * [Inactive]   - Wait for the activity to go above the wake threshold (TH1)
     *              - Activity moves the routine into the active state.
     *
     * [Active]     - Start recording measurements to FIFO (if in a triggered mode)
     *              - Loop back to 2 while activity stays above the inactive threshold (TH2)
     *              - If activity drops below the inactive threshold, the device moves to the pre-sleep stage
     *
     * [Pre-sleep]  - Data continues to be recorded to the FIFO in this state
     *              - A minimum number of samples are recorded after entering pre-sleep before the device returns to the
     *                   inactive state.
     *              - If activity has restarted, then the active state will be triggered once more.
     */

    // Inactive state
    sleep_settings.code[0] = LIS3DSH_NOP << 4 | LIS3DSH_GTTH1; // Wait until wake threshold triggered

    // Active state
    sleep_settings.code[1] = LIS3DSH_SRP;                        // Set the new reset loop point to make an active loop
    sleep_settings.code[2] = LIS3DSH_OUTC;                       // Set INT2 high for FIFO purposes
    sleep_settings.code[3] = LIS3DSH_GNTH2 << 4 | LIS3DSH_LNTH2; // Reset while activity continues

    // Pre-sleep state
    sleep_settings.code[4] = LIS3DSH_CRP; // Remove the reset loop
    // INT2 stays high when the final state is reached, so FIFO will continue to record
    sleep_settings.code[5] = LIS3DSH_TI1 << 4 | LIS3DSH_NOP; // Reset once timer reaches minimum reads

    sm_settings[1] = sleep_settings;
    configure_state_machines();
}
