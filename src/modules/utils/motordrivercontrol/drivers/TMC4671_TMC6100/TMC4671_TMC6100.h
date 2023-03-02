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

    TMC4671_TMC6100(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi, char designator);

    void init(uint16_t cs);
    int set_microsteps(int number_of_steps);
    int get_microsteps(void);

    void setStepInterpolation(int8_t value);
    void setDoubleEdge(int8_t value);
    void set_current(uint16_t current);
    unsigned int get_current(void);
    int8_t getOverTemperature(void);

    bool isStandStill(void);

    void set_enable(bool enabled);

    bool isEnabled();

    void readStatus(int8_t read_value);

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

    unsigned int pid_torque_p_i;
    unsigned int pid_flux_p_i;
    unsigned int pid_velocity_p_i;
    unsigned int pid_position_p_i;

    unsigned int adc_mclk_a;
    unsigned int adc_mclk_b;

    unsigned int adc_mdec_b_a;
    unsigned int adc_i0_offset;
    unsigned int adc_i1_offset;

    char designator;
    
    bool write_only = false;
};

