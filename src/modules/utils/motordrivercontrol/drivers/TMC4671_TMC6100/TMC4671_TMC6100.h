/*
 modified from...
 TMC4671.cpp - - TMC4671 Stepper library for Wiring/Arduino

 based on the stepper library by Tom Igoe, et. al.

 Copyright (c) 2011, Interactive Matter, Marcus Nowotny

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.

 */


// ensure this library description is only included once
#pragma once

#include <functional>
#include <map>
#include <bitset>

#include "../StepperDrv.h"
#include "Pin.h"

class StreamOutput;

/*!
 * \class TMC4671
 * \brief Class representing a TMC4671 stepper driver
 */
class TMC4671_TMC6100 : public StepperDrv 
{
public:
    /*!
     * \brief creates a new represenatation of a stepper motor connected to a TMC4671 stepper driver
     *
     * This is the main constructor. If in doubt use this. You must provide all parameters as described below.
     *
     * \param spi send function
     *
     * By default the Constant Off Time chopper is used, see TCM262Stepper.setConstantOffTimeChopper() for details.
     * This should work on most motors (YMMV). You may want to configure and use the Spread Cycle Chopper, see  setSpreadCycleChopper().
     *
     * By default a microstepping of 1/32th is used to provide a smooth motor run, while still giving a good progression per step.
     * You can select a different stepping with setMicrosteps() to aa different value.
     * \sa start(), setMicrosteps()
     */
    TMC4671_TMC6100(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi, char designator);

    /*!
     * \brief configures the TMC4671 stepper driver. Before you called this function the stepper driver is in nonfunctional mode.
     *
     * \param rms_current the maximum current to privide to the motor in mA (!). A value of 200 would send up to 200mA to the motor

     * This routine configures the TMC4671 stepper driver for the given values via SPI.
     * Most member functions are non functional if the driver has not been started.
     * Therefore it is best to call this in your Arduino setup() function.
     */
    void init(uint16_t cs);

    /*!
     * \brief Set the number of microsteps in 2^i values (rounded) up to 256
     *
     * This method set's the number of microsteps per step in 2^i interval.
     * This means you can select 1, 2, 4, 16, 32, 64, 128 or 256 as valid microsteps.
     * If you give any other value it will be rounded to the next smaller number (3 would give a microstepping of 2).
     * You can always check the current microstepping with getMicrosteps().
     */
    int set_microsteps(int number_of_steps);

    /*!
     * \brief returns the effective current number of microsteps selected.
     *
     * This function always returns the effective number of microsteps.
     * This can be a bit different than the micro steps set in setMicrosteps() since it is rounded to 2^i.
     *
     * \sa set_microsteps()
     */
    int get_microsteps(void);

    void setStepInterpolation(int8_t value);
    void setDoubleEdge(int8_t value);

    /*!
     * \brief set the maximum motor current in mA (1000 is 1 Amp)
     * Keep in mind this is the maximum peak Current. The RMS current will be 1/sqrt(2) smaller. The actual current can also be smaller
     * by employing CoolStep.
     * \param current the maximum motor current in mA
     * \sa getCurrent(), getCurrentCurrent()
     */
    void set_current(uint16_t current);

    /*!
     * \brief readout the motor maximum current in mA (1000 is an Amp)
     * This is the maximum current. to get the current current - which may be affected by CoolStep us getCurrentCurrent()
     *\return the maximum motor current in milli amps
     * \sa getCurrentCurrent()
     */
    unsigned int get_current(void);

    /*!
     * \brief Return over temperature status of the last status readout
     * return 0 is everything is OK, TMC4671_OVERTEMPERATURE_PREWARING if status is reached, TMC4671_OVERTEMPERATURE_SHUTDOWN is the chip is shutdown, -1 if the status is unknown.
     * Keep in mind that this method does not enforce a readout but uses the value of the last status readout.
     * You may want to use getMotorPosition() or getCurrentStallGuardReading() to enforce an updated status readout.
     */
    int8_t getOverTemperature(void);

    /*!
     * \brief Is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
     * \return true is yes, false if not.
     * Keep in mind that this method does not enforce a readout but uses the value of the last status readout.
     * You may want to use getMotorPosition() or getCurrentStallGuardReading() to enforce an updated status readout.
     */
    bool isStandStill(void);

    /*!
     *\brief enables or disables the motor driver bridges. If disabled the motor can run freely. If enabled not.
     *\param enabled a bool value true if the motor should be enabled, false otherwise.
     */
    void set_enable(bool enabled);

    /*!
     *\brief checks if the output bridges are enabled. If the bridges are not enabled the motor can run freely
     *\return true if the bridges and by that the motor driver are enabled, false if not.
     *\sa setEnabled()
     */
    bool isEnabled();

    /*!
     * \brief Manually read out the status register
     * This function sends a byte to the motor driver in order to get the current readout. The parameter read_value
     * seletcs which value will get returned. If the read_vlaue changes in respect to the previous readout this method
     * automatically send two bytes to the motor: one to set the redout and one to get the actual readout. So this method
     * may take time to send and read one or two bits - depending on the previous readout.
     * \param read_value selects which value to read out (0..3). You can use the defines TMC4671_READOUT_POSITION, TMC_262_READOUT_STALLGUARD, or TMC_262_READOUT_CURRENT
     * \sa TMC4671_READOUT_POSITION, TMC_262_READOUT_STALLGUARD, TMC_262_READOUT_CURRENT
     */
    void readStatus(int8_t read_value);

    /*!
     * \brief Prints out all the information that can be found in the last status read out - it does not force a status readout.
     * The result is printed via Serial
     */
    void dump_status(StreamOutput *stream);
    bool set_raw_register(StreamOutput *stream, uint32_t reg, uint32_t val);
    bool check_alarm();

    using options_t= std::map<char,int>;

    bool set_options(const options_t& options);

private:
    //helper routione to get the top 10 bit of the readout
    inline int getReadoutValue();
    bool check_error_status_bits(StreamOutput *stream);

    // SPI sender
    inline void send262(uint8_t device, uint8_t reg, uint32_t datagram);
    std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi;

    //driver control register copies to easily set & modify the registers
    unsigned long driver_control_register_value;
    unsigned long chopper_config_register;
    unsigned long cool_step_register_value;
    unsigned long stall_guard2_current_register_value;
    unsigned long driver_configuration_register_value;
    //the driver status result
    unsigned long driver_status_result;

    //status values
    int microsteps; //the current number of micro steps

    std::bitset<8> error_reported;

    // only beeded for the tuning app report
    struct {
        int8_t blank_time:8;
        int8_t constant_off_time:5; //we need to remember this value in order to enable and disable the motor
        int8_t h_start:4;
        int8_t h_end:4;
        int8_t h_decrement:3;
        bool started:1; //if the stepper has been started yet
    };

    Pin *spi_cs_ctrl_pin;
    Pin *spi_cs_drv_pin;
    Pin *enable_pin;

    char designator;
    
    bool write_only = false;
};

