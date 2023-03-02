/*
 Highly modifed from....

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

#include "TMC4671_TMC6100.h"
#include "TMC4671_Constants.h"
#include "TMC4671_Fields.h"
#include "TMC4671_Register.h"
#include "TMC6100_Constants.h"
#include "TMC6100_Fields.h"
#include "TMC6100_Register.h"
#include "mbed.h"
#include "StreamOutput.h"
#include "Kernel.h"
#include "libs/StreamOutputPool.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "ConfigValue.h"
#include "Config.h"
#include "checksumm.h"
#include "StepTicker.h"
#include "utils.h"

#define motor_driver_control_checksum   CHECKSUM("motor_driver_control")
#define enable_pin_checksum             CHECKSUM("enable_pin")

#define spi_cs_ctrl_pin_checksum        CHECKSUM("spi_cs_ctrl_pin")
#define spi_cs_drv_pin_checksum         CHECKSUM("spi_cs_drv_pin")
#define pid_torque_p_i_checksum         CHECKSUM("pid_torque_p_i")
#define pid_flux_p_i_checksum           CHECKSUM("pid_flux_p_i")
#define pid_velocity_p_i_checksum       CHECKSUM("pid_velocity_p_i")
#define pid_position_p_i_checksum       CHECKSUM("pid_position_p_i")

#define adc_mclk_a_checksum             CHECKSUM("adc_mclk_a")
#define adc_mclk_b_checksum             CHECKSUM("adc_mclk_b")

#define adc_mdec_b_a_checksum           CHECKSUM("adc_mdec_b_a")
#define adc_i0_offset_checksum          CHECKSUM("adc_i0_offset")
#define adc_i1_offset_checksum          CHECKSUM("adc_i1_offset")


#define SPI_DEV_DRV     0
#define SPI_DEV_CTRL    1

/*
 * Constructor
 */
TMC4671_TMC6100::TMC4671_TMC6100(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi, char d) : spi(spi), designator(d)
{
    connection_method = DriveParameters::SPI;
    max_current= 3000;
    
    //we are not started yet
    started = false;
    error_reported.reset();
}

/*
 * configure the stepper driver
 * just must be called.
 */
void TMC4671_TMC6100::init(uint16_t cs)
{
    spi_cs_ctrl_pin = new Pin();
    spi_cs_ctrl_pin->from_string(THEKERNEL->config->value( motor_driver_control_checksum, cs, spi_cs_ctrl_pin_checksum)->by_default("nc")->as_string())->as_output();

    spi_cs_drv_pin = new Pin();
    spi_cs_drv_pin->from_string(THEKERNEL->config->value( motor_driver_control_checksum, cs, spi_cs_drv_pin_checksum)->by_default("nc")->as_string())->as_output();


    enable_pin = new Pin();
    enable_pin->from_string(THEKERNEL->config->value( motor_driver_control_checksum, cs, enable_pin_checksum)->by_default("nc")->as_string())->as_output();

    this->pid_torque_p_i= THEKERNEL->config->value(motor_driver_control_checksum, cs, pid_torque_p_i_checksum)->required()->as_uint();
    this->pid_flux_p_i= THEKERNEL->config->value(motor_driver_control_checksum, cs, pid_flux_p_i_checksum)->required()->as_uint();
    this->pid_velocity_p_i= THEKERNEL->config->value(motor_driver_control_checksum, cs, pid_velocity_p_i_checksum)->required()->as_uint();
    this->pid_position_p_i= THEKERNEL->config->value(motor_driver_control_checksum, cs, pid_position_p_i_checksum)->required()->as_uint();
    this->adc_mclk_a= THEKERNEL->config->value(motor_driver_control_checksum, cs, adc_mclk_a_checksum)->required()->as_uint();
    this->adc_mclk_b= THEKERNEL->config->value(motor_driver_control_checksum, cs, adc_mclk_b_checksum)->required()->as_uint();
    this->adc_mdec_b_a= THEKERNEL->config->value(motor_driver_control_checksum, cs, adc_mdec_b_a_checksum)->required()->as_uint();
    this->adc_i0_offset= THEKERNEL->config->value(motor_driver_control_checksum, cs, adc_i0_offset_checksum)->required()->as_uint();
    this->adc_i1_offset= THEKERNEL->config->value(motor_driver_control_checksum, cs, adc_i1_offset_checksum)->required()->as_uint();

    if(!spi_cs_ctrl_pin->connected()) {
        printf("MotorDriverControl ERROR: control chip select not defined\n");
        return; // if not defined then we can't use this instance
    }
    spi_cs_ctrl_pin->set(1);

    if(!spi_cs_drv_pin->connected()) {
        printf("MotorDriverControl ERROR: drive chip select not defined\n");
        return; // if not defined then we can't use this instance
    }
    spi_cs_drv_pin->set(1);

    if(!enable_pin->connected()) {
        printf("MotorDriverControl ERROR: enable pin not defined\n");
        return; // if not defined then we can't use this instance
    }
    enable_pin->set(0);


    send262(SPI_DEV_DRV, 0x00, (0 << TMC6100_DISABLE_SHIFT)| // Enable
        (0 << TMC6100_SINGLELINE_SHIFT)   | // Use individual L+H signals
        (1 << TMC6100_FAULTDIRECT_SHIFT)  | // Fault output shows each protective action
        (1 << TMC6100_CURRENT_ZERO_SHIFT)   // Disable current amplifier)
    );
    
    // Motor type & pole pairs
    send262(SPI_DEV_CTRL, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030002); //0x00030002

    // PWM configuration
    send262(SPI_DEV_CTRL, TMC4671_PWM_POLARITIES, 0x00000000);
    send262(SPI_DEV_CTRL, TMC4671_PWM_MAXCNT, 0x00000F9F);
    send262(SPI_DEV_CTRL, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
    send262(SPI_DEV_CTRL, TMC4671_PWM_SV_CHOP, 0x00000007);
    
    // ADC configuration
    send262(SPI_DEV_CTRL, TMC4671_ADC_I_SELECT, 0x09000100);
    send262(SPI_DEV_CTRL, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
    send262(SPI_DEV_CTRL, TMC4671_dsADC_MCLK_A, this->adc_mclk_a);
    send262(SPI_DEV_CTRL, TMC4671_dsADC_MCLK_B, this->adc_mclk_b);
    send262(SPI_DEV_CTRL, TMC4671_dsADC_MDEC_B_MDEC_A, this->adc_mdec_b_a);
    send262(SPI_DEV_CTRL, TMC4671_ADC_I0_SCALE_OFFSET, this->adc_i0_offset);
    send262(SPI_DEV_CTRL, TMC4671_ADC_I1_SCALE_OFFSET, this->adc_i1_offset);
    
    // ABN encoder settings
    send262(SPI_DEV_CTRL, TMC4671_ABN_DECODER_MODE, 0x00000000);
    send262(SPI_DEV_CTRL, TMC4671_ABN_DECODER_PPR, 0x00003600);
    send262(SPI_DEV_CTRL, TMC4671_ABN_DECODER_COUNT, 0x00000000);
    send262(SPI_DEV_CTRL, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
    
    // Limits
    send262(SPI_DEV_CTRL, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000007D0);
    send262(SPI_DEV_CTRL, TMC4671_PID_VELOCITY_LIMIT, 0x000003E8);
    send262(SPI_DEV_CTRL, TMC4671_PID_ACCELERATION_LIMIT, 0x00002710);
    
    // PI settings
    send262(SPI_DEV_CTRL, TMC4671_PID_TORQUE_P_TORQUE_I, this->pid_torque_p_i);
    send262(SPI_DEV_CTRL, TMC4671_PID_FLUX_P_FLUX_I, this->pid_flux_p_i);
    send262(SPI_DEV_CTRL, TMC4671_PID_VELOCITY_P_VELOCITY_I, this->pid_velocity_p_i);
    send262(SPI_DEV_CTRL, TMC4671_PID_POSITION_P_POSITION_I, this->pid_position_p_i);

    // Init encoder (mode 0)
    send262(SPI_DEV_CTRL, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
    send262(SPI_DEV_CTRL, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
    send262(SPI_DEV_CTRL, TMC4671_PHI_E_SELECTION, 0x00000001);
    send262(SPI_DEV_CTRL, TMC4671_PHI_E_EXT, 0x00000000);
    send262(SPI_DEV_CTRL, TMC4671_UQ_UD_EXT, 0x00000FD0);
    safe_delay_ms(1000);
    
    send262(SPI_DEV_CTRL, TMC4671_PID_POSITION_ACTUAL, 0x00000000);
    send262(SPI_DEV_CTRL, TMC4671_PID_POSITION_TARGET, 0x00000000); //Also set target pos to avoid unexpected movements
    send262(SPI_DEV_CTRL, TMC4671_ABN_DECODER_COUNT, 0x00000000);
    safe_delay_ms(1000);
    
    // Feedback selection
    send262(SPI_DEV_CTRL, TMC4671_PHI_E_SELECTION, 0x00000003);
    send262(SPI_DEV_CTRL, TMC4671_VELOCITY_SELECTION, 0x00000009);
    send262(SPI_DEV_CTRL, TMC4671_POSITION_SELECTION, 0x00000009);
    printf("MotorDriverControl INFO: Control feedback selection configuration written\n");

    
    started = true;

    // set_enable(false);

    // //set a nice microstepping value
    // set_microsteps(DEFAULT_MICROSTEPPING_VALUE);

}

void TMC4671_TMC6100::set_current(uint16_t current)
{
    //TODO: implement set current
}

unsigned int TMC4671_TMC6100::get_current(void)
{
    return (unsigned int)0;
}


/*
 * Set the number of microsteps per step.
 * 0,2,4,8,16,32,64,128,256 is supported
 * any value in between will be mapped to the next smaller value
 * 0 and 1 set the motor in full step mode
 */
int TMC4671_TMC6100::set_microsteps(int number_of_steps)
{
    long setting_pattern;
    //poor mans log
    if (number_of_steps >= 256) {
        setting_pattern = 0;
        microsteps = 256;
    } else if (number_of_steps >= 128) {
        setting_pattern = 1;
        microsteps = 128;
    } else if (number_of_steps >= 64) {
        setting_pattern = 2;
        microsteps = 64;
    } else if (number_of_steps >= 32) {
        setting_pattern = 3;
        microsteps = 32;
    } else if (number_of_steps >= 16) {
        setting_pattern = 4;
        microsteps = 16;
    } else if (number_of_steps >= 8) {
        setting_pattern = 5;
        microsteps = 8;
    } else if (number_of_steps >= 4) {
        setting_pattern = 6;
        microsteps = 4;
    } else if (number_of_steps >= 2) {
        setting_pattern = 7;
        microsteps = 2;
        //1 and 0 lead to full step
    } else if (number_of_steps <= 1) {
        setting_pattern = 8;
        microsteps = 1;
    }

    //delete the old value
    this->driver_control_register_value &= 0xFFFF0ul;
    //set the new value
    this->driver_control_register_value |= setting_pattern;

    //if started we directly send it to the motor
    // if (started) {
    //     send262(driver_control_register_value);
    // }
    
    return get_microsteps();
}

/*
 * returns the effective number of microsteps at the moment
 */
int TMC4671_TMC6100::get_microsteps(void)
{
    return microsteps;
}

void TMC4671_TMC6100::setStepInterpolation(int8_t value)
{
    // if (value) {
    //     driver_control_register_value |= STEP_INTERPOLATION;
    // } else {
    //     driver_control_register_value &= ~(STEP_INTERPOLATION);
    // }
    // //if started we directly send it to the motor
    // if (started) {
    //     send262(driver_control_register_value);
    // }
}

void TMC4671_TMC6100::set_enable(bool enabled)
{
    enable_pin->set(enabled);
    //enable handled via setting pin in motorDriverControl.
    //Read status feedback from device.
}

bool TMC4671_TMC6100::isEnabled()
{
    return enable_pin->get();
}

/*
 * reads a value from the TMC4671 status register. The value is not obtained directly but can then
 * be read by the various status routines.
 *
 */
void TMC4671_TMC6100::readStatus(int8_t read_value)
{
    // unsigned long old_driver_configuration_register_value = driver_configuration_register_value;
    // //reset the readout configuration
    // driver_configuration_register_value &= ~(READ_SELECTION_PATTERN);
    // //this now equals TMC4671_READOUT_POSITION - so we just have to check the other two options
    // if (read_value == TMC4671_READOUT_STALLGUARD) {
    //     driver_configuration_register_value |= READ_STALL_GUARD_READING;
    // } else if (read_value == TMC4671_READOUT_CURRENT) {
    //     driver_configuration_register_value |= READ_STALL_GUARD_AND_COOL_STEP;
    // }
    // //all other cases are ignored to prevent funny values
    // //check if the readout is configured for the value we are interested in
    // if (driver_configuration_register_value != old_driver_configuration_register_value) {
    //     //because then we need to write the value twice - one time for configuring, second time to get the value, see below
    //     send262(driver_configuration_register_value);
    // }
    // //write the configuration to get the last status
    // send262(driver_configuration_register_value);
}

/*
 returns if there is any over temperature condition:
 OVER_TEMPERATURE_PREWARING if pre warning level has been reached
 OVER_TEMPERATURE_SHUTDOWN if the temperature is so hot that the driver is shut down
 Any of those levels are not too good.
*/
int8_t TMC4671_TMC6100::getOverTemperature(void)
{
     return 0;
}

//is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
bool TMC4671_TMC6100::isStandStill(void)
{
    // if (!this->started) {
        return false;
    // }
    // return (driver_status_result & STATUS_STAND_STILL);
}


//reads the stall guard setting from last status
//returns -1 if stallguard inforamtion is not present
int TMC4671_TMC6100::getReadoutValue(void)
{
    return (int)(driver_status_result >> 10);
}

void TMC4671_TMC6100::dump_status(StreamOutput *stream)
{
    // if (!this->write_only) {
    //     stream->printf("designator %c, Chip type TMC4671\n", designator);

    //     check_error_status_bits(stream);

    //     if (this->isStandStill()) {
    //         stream->printf("INFO: Motor is standing still.\n");
    //     }

    //     int value = getReadoutValue();
    //     stream->printf("Microstep position phase A: %d\n", value);

    //     stream->printf("Current setting: %dmA\n", get_current());

    //     stream->printf("Microsteps: 1/%d\n", get_microsteps());

    //     stream->printf("Register dump:\n");
    //     stream->printf(" driver control register: %08lX(%ld)\n", driver_control_register_value, driver_control_register_value);
    //     stream->printf(" chopper config register: %08lX(%ld)\n", chopper_config_register, chopper_config_register);
    //     stream->printf(" cool step register: %08lX(%ld)\n", cool_step_register_value, cool_step_register_value);
    //     stream->printf(" stall guard2 current register: %08lX(%ld)\n", stall_guard2_current_register_value, stall_guard2_current_register_value);
    //     stream->printf(" driver configuration register: %08lX(%ld)\n", driver_configuration_register_value, driver_configuration_register_value);
    //     stream->printf(" motor_driver_control.xxx.reg %05lX,%05lX,%05lX,%05lX,%05lX\n", driver_control_register_value, chopper_config_register, cool_step_register_value, stall_guard2_current_register_value, driver_configuration_register_value);

    // } else {
    //     // TODO hardcoded for X need to select ABC as needed
    //     bool moving = THEROBOT->actuators[0]->is_moving();
    //     // dump out in the format that the processing script needs
       
    //     stream->printf("d%d,", THEROBOT->actuators[0]->which_direction() ? -1 : 1);
    //     stream->printf("c%u,m%d,", get_current(), get_microsteps());


    //     //detect the winding status
    //     if (isOpenLoadA()) {
    //         stream->printf("ao,");
    //     } else if(isShortToGroundA()) {
    //         stream->printf("ag,");
    //     } else {
    //         stream->printf("a-,");
    //     }
    //     //detect the winding status
    //     if (isOpenLoadB()) {
    //         stream->printf("bo,");
    //     } else if(isShortToGroundB()) {
    //         stream->printf("bg,");
    //     } else {
    //         stream->printf("b-,");
    //     }

    //     if (isEnabled()) {
    //         stream->printf("e1,");
    //     } else {
    //         stream->printf("e0,");
    //     }

    //     //write out the current chopper config
    //     stream->printf("Cm%d,", (chopper_config_register & CHOPPER_MODE_T_OFF_FAST_DECAY) != 0);
    //     stream->printf("Co%d,Cb%d,", constant_off_time, blank_time);
    //     if ((chopper_config_register & CHOPPER_MODE_T_OFF_FAST_DECAY) == 0) {
    //         stream->printf("Cs%d,Ce%d,Cd%d,", h_start, h_end, h_decrement);
    //     }
    //     stream->printf("\n");
    // }
}

// check error bits and report, only report once
bool TMC4671_TMC6100::check_error_status_bits(StreamOutput *stream)
{
    bool error= false;
    // readStatus(TMC4671_READOUT_POSITION); // get the status bits

    // if (this->getOverTemperature()&TMC4671_OVERTEMPERATURE_SHUTDOWN) {
    //     if(!error_reported.test(1)) stream->printf("%c - ERROR: Overtemperature Shutdown!\n", designator);
    //     error=true;
    //     error_reported.set(1);
    // }else{
    //     error_reported.reset(1);
    // }

    // if (this->isShortToGroundA()) {
    //     if(!error_reported.test(2)) stream->printf("%c - ERROR: SHORT to ground on channel A!\n", designator);
    //     error=true;
    //     error_reported.set(2);
    // }else{
    //     error_reported.reset(2);
    // }

    // if (this->isShortToGroundB()) {
    //     if(!error_reported.test(3)) stream->printf("%c - ERROR: SHORT to ground on channel B!\n", designator);
    //     error=true;
    //     error_reported.set(3);
    // }else{
    //     error_reported.reset(3);
    // }

    // // these seem to be triggered when moving so ignore them for now
    // if (this->isOpenLoadA()) {
    //     if(!error_reported.test(4)) stream->printf("%c - ERROR: Channel A seems to be unconnected!\n", designator);
    //     error=true;
    //     error_reported.set(4);
    // }else{
    //     error_reported.reset(4);
    // }

    // if (this->isOpenLoadB()) {
    //     if(!error_reported.test(5)) stream->printf("%c - ERROR: Channel B seems to be unconnected!\n", designator);
    //     error=true;
    //     error_reported.set(5);
    // }else{
    //     error_reported.reset(5);
    // }

    // // if(error) {
    // //     stream->printf("%08X\n", driver_status_result);
    // // }
    return error;
}

bool TMC4671_TMC6100::check_alarm()
{
    return check_error_status_bits(THEKERNEL->streams);
}

// sets a raw register to the value specified, for advanced settings
// register 255 writes them, 0 displays what registers are mapped to what
// FIXME status registers not reading back correctly, check docs
bool TMC4671_TMC6100::set_raw_register(StreamOutput *stream, uint32_t reg, uint32_t val)
{
    // switch(reg) {
    //     case 255:
    //         send262(driver_control_register_value);
    //         send262(chopper_config_register);
    //         send262(cool_step_register_value);
    //         send262(stall_guard2_current_register_value);
    //         send262(driver_configuration_register_value);
    //         stream->printf("Registers written\n");
    //         break;


    //     case 1: driver_control_register_value = val; stream->printf("driver control register set to %08lX\n", val); break;
    //     case 2: chopper_config_register = val; stream->printf("chopper config register set to %08lX\n", val); break;
    //     case 3: cool_step_register_value = val; stream->printf("cool step register set to %08lX\n", val); break;
    //     case 4: stall_guard2_current_register_value = val; stream->printf("stall guard2 current register set to %08lX\n", val); break;
    //     case 5: driver_configuration_register_value = val; stream->printf("driver configuration register set to %08lX\n", val); break;

    //     default:
    //         stream->printf("1: driver control register\n");
    //         stream->printf("2: chopper config register\n");
    //         stream->printf("3: cool step register\n");
    //         stream->printf("4: stall guard2 current register\n");
    //         stream->printf("5: driver configuration register\n");
    //         stream->printf("255: update all registers\n");
    //         return false;
    // }
    return true;
}

/*
 * send register settings to the stepper driver via SPI
 * returns the current status
 * sends 40bits, the last 20 bits of the 24bits is taken as the command
 */
void TMC4671_TMC6100::send262(uint8_t device, uint8_t reg, uint32_t datagram)
{

    uint8_t buf[] {
        reg | 0x80, 
        (uint8_t)(datagram >> 24),
        (uint8_t)(datagram >> 16),
        (uint8_t)(datagram >> 8),
        (uint8_t)(datagram >> 0)
    };
    uint8_t rbuf[5];

    device == SPI_DEV_DRV ? spi_cs_drv_pin->set(0) : spi_cs_ctrl_pin->set(0);
    //write/read the values
    spi(buf, 5, rbuf);

    device == SPI_DEV_DRV ? spi_cs_drv_pin->set(1) : spi_cs_ctrl_pin->set(1);
    // construct reply
    //unsigned long i_datagram = ((rbuf[0] << 16) | (rbuf[1] << 8) | (rbuf[2])) >> 4;

    //store the datagram as status result
    //driver_status_result = i_datagram;

    //THEKERNEL->streams->printf("sent: %02X, %02X, %02X received: %02X, %02X, %02X \n", buf[0], buf[1], buf[2], rbuf[0], rbuf[1], rbuf[2]);
}

#define HAS(X) (options.find(X) != options.end())
#define GET(X) (options.at(X))
bool TMC4671_TMC6100::set_options(const options_t& options)
{
    bool set = false;
    return set;
}
