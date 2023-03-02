/*
 * TMC4671_Registers.h
 *
 *  Created on: 18.02.2016
 *      Author: ed
 */

#ifndef TMC4671_REGISTERS_H
#define TMC4671_REGISTERS_H

	// ===== TMC4671 register set =====

	#define TMC4671_CHIPINFO_DATA                      0x00
	#define TMC4671_CHIPINFO_ADDR                      0x01

	#define TMC4671_ADC_RAW_DATA                       0x02
	#define TMC4671_ADC_RAW_ADDR                       0x03

	/*
		Bit 20: sel_nclk_mclk_i_b (0: off, 1: on)
		Bit 19: mdat_polarity_b (0: off, 1: on)
		Bit 18: mclk_polarity_b (0: off, 1: on)
		Bit 17-16: cfg_dsmodulator_b
			0: int. dsMOD
			1: ext. MCLK input
			2: ext. MCLK output
			3: ext. CMP
		Bit 4: sel_nclk_mclk_i_a (0: off, 1: on)
		Bit 3: mdat_polarity_a (0: off, 1: on)
		Bit 2: mclk_polarity_a (0: off, 1: on)
		Bit 1-0: cfg_dsmodulator_a
			0: int. dsMOD
			1: ext. MCLK input
			2: ext. MCLK output
			3: ext. CMP
	*/
	#define TMC4671_dsADC_MCFG_B_MCFG_A                0x04

	/*
		Bit 31-0: dsADC_MCLK_A; dsADC_MCLK_A = (2^31xfMCLK)/fCLK
	*/
	#define TMC4671_dsADC_MCLK_A                       0x05
	/*
		Bit 31-0: dsADC_MCLK_B; dsADC_MCLK_B = (2^31xfMCLK)/fCLK
	*/
	#define TMC4671_dsADC_MCLK_B                       0x06

	/*
		Bit 31-16: dsADC_MDEC_B; 0: PWM synchronous, others according to register content
		Bit 15-0: dsADC_MDEC_A; 0: PWM synchronous, others according to register content
	*/
	#define TMC4671_dsADC_MDEC_B_MDEC_A                0x07

	/*
		Bit 31-16: ADC_I1_SCALE, Scaling factor for current ADC channel 1.
		Bit 15-0: ADC_I1_OFFSET, Offset for current ADC channel 1.
	*/
	#define TMC4671_ADC_I1_SCALE_OFFSET                0x08
	/*
		Bit 31-16: ADC_I0_SCALE; Scaling factor for current ADC channel 0.
		Bit 15-0: ADC_I0_OFFSET; Offset for current ADC channel 0.
	*/
	#define TMC4671_ADC_I0_SCALE_OFFSET                0x09

	/*
		Bit 29-28: ADC_I_WY_SELECT
			0: WY = ADC_I0
			1: WY = ADC_I1
			2: WY = ADC_I2 (default)
		Bit 27-26: ADC_I_V_SELECT
			0: V = ADC_I0
			1: V = ADC_I1 (default)
			2: V = ADC_I2
		Bit 25-24: ADC_I_UX_SELECT
			0: UX = ADC_I0 (default)
			1: UX = ADC_I1
			2: UX = ADC_I2
		Bit 15-8: ADC_I1_SELECT, Select input for raw current ADC_I1_RAW.
			0: ADCSD_I0_RAW (sigma delta ADC)
			1: ADCSD_I1_RAW (sigma delta ADC)
			2: ADC_I0_EXT (from register)
			3: ADC_I1_EXT (from register)
		Bit 7-0: ADC_I0_SELECT, Select input for raw current ADC_I0_RAW.
			0: ADCSD_I0_RAW (sigma delta ADC)
			1: ADCSD_I1_RAW (sigma delta ADC)
			2: ADC_I0_EXT (from register)
			3: ADC_I1_EXT (from register)
	*/
	#define TMC4671_ADC_I_SELECT                       0x0A
	#define TMC4671_ADC_I1_I0_EXT                      0x0B

	#define TMC4671_DS_ANALOG_INPUT_STAGE_CFG          0x0C

	#define TMC4671_AENC_0_SCALE_OFFSET                0x0D
	#define TMC4671_AENC_1_SCALE_OFFSET                0x0E
	#define TMC4671_AENC_2_SCALE_OFFSET                0x0F

	#define TMC4671_AENC_SELECT                        0x11

	#define TMC4671_ADC_IWY_IUX                        0x12
	#define TMC4671_ADC_IV                             0x13
	#define TMC4671_AENC_WY_UX                         0x15
	#define TMC4671_AENC_VN                            0x16

	/*
        Bit 1: High Side gate control (0: off, 1: on)
        bit 0: Low Side gate control (0: off, 1: on)
    */
	#define TMC4671_PWM_POLARITIES                     0x17

	/*
		PWM maximum (count-1), PWM frequency is fPWM[Hz] = 100MHz/(PWM_MAXCNT+1)
	*/
	#define TMC4671_PWM_MAXCNT                         0x18

	/*
		Bit 15-8: Break Before Make time tBBM_H[10ns] for high side MOS-FET gate control
		Bit 7-0: Break Before Make time tBBM_L[10ns] for low side MOS-FET gate control
	*/
	#define TMC4671_PWM_BBM_H_BBM_L                    0x19

	/*
		Bit 8: use Space Vector PWM
			0: Space Vector PWM disabled
			1: Space Vector PWM enabled
		Bit 7-0: PWM chopper mode, defining how to chopper:
			0: off, free running
			1: off, low side permanent = ON
			2: off, high side permanent = ON
			3: off, free running
			4: off, free running
			5: low side chopper, high side off
			6: high side chopper, low side off
			7: centered PWM for FOC
	*/
	#define TMC4671_PWM_SV_CHOP                        0x1A

	/*
        bit 23-16: Motor Type
            0: No motor
            1: Single phase DC
            2: Two phase Stepper
            3: Three phase BLDC
        bit 15-0: Pole Pairs
    */
	#define TMC4671_MOTOR_TYPE_N_POLE_PAIRS            0x1B

	/*
		Bit 15-0: PHI_E_EXT, Electrical angle phi_e_ext for external writing into this register.
	*/
	#define TMC4671_PHI_E_EXT                          0x1C
	#define TMC4671_PHI_M_EXT                          0x1D
	#define TMC4671_POSITION_EXT                       0x1E
	#define TMC4671_OPENLOOP_MODE                      0x1F
	#define TMC4671_OPENLOOP_ACCELERATION              0x20
	#define TMC4671_OPENLOOP_VELOCITY_TARGET           0x21
	#define TMC4671_OPENLOOP_VELOCITY_ACTUAL           0x22
	#define TMC4671_OPENLOOP_PHI                       0x23
	/*
		Bit 31-16: UQ_EXT, External writable parameter for open loop voltage control mode, usefull during system setup, U_Q component.
		Bit 15-0: UD_EXT, External writable parameter for open loop voltage control mode, usefull during system setup, U_D component.
	*/
	#define TMC4671_UQ_UD_EXT                          0x24

	/*
		Bit 12: Decoder count direction. (0: positive, 1: negative)
		Bit 8: Write direction at Npulse event between ABN_DECODER_COUNT_N and ABN_DECODER_COUNT. (0: COUNT => COUNT_N, 1: COUNT_N => COUNT)
		Bit 3: N and A and B (0: Ignore A and B polarity with, Npulse = N, 1: Npulse = N and A and B)
		Bit 2: Polarity of N pulse. (0: off, 1: on)
		Bit 1: Polarity of B pulse. (0: off, 1: on)
		Bit 0: Polarity of A pulse. (0: off, 1: on)

	*/
	#define TMC4671_ABN_DECODER_MODE                   0x25

	/*
		Bit 23-0: ABN_DECODER_PPR, Decoder pulses per mechanical revolution.
	*/
	#define TMC4671_ABN_DECODER_PPR                    0x26
	/*
		Bit 23-0: ABN_DECODER_COUNT, Raw decoder count; the digital decoder engine counts modulo (decoder_ppr).
	*/
	#define TMC4671_ABN_DECODER_COUNT                  0x27

	/*
		Bit 23-0: ABN_DECODER_COUNT_N, Decoder count latched on N pulse, when N pulse clears decoder_count also decoder_count_n is 0.
	*/
	#define TMC4671_ABN_DECODER_COUNT_N                0x28
	/*
		Bit 31-16: ABN_DECODER_PHI_E_OFFSET, ABN_DECODER_PHI_E_OFFSET to shift (rotate) angle DECODER_PHI_E.
		Bit 15-0: ABN_DECODER_PHI_M_OFFSET, ABN_DECODER_PHI_M_OFFSET to shift (rotate) angle DECODER_PHI_M.
	*/
	#define TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET     0x29
	/*
		Bit 31-16: ABN_DECODER_PHI_E, ABN_DECODER_PHI_E = (ABN_DECODER_PHI_M * N_POLE_PAIRS_) + ABN_DECODER_PHI_E_OFFSET
		Bit 15-0: ABN_DECODER_PHI_M, ABN_DECODER_PHI_M = ABN_DECODER_COUNT * 2^16 / ABN_DECODER_PPR + ABN_DECODER_PHI_M_OFFSET;
	*/
	#define TMC4671_ABN_DECODER_PHI_E_PHI_M            0x2A

	#define TMC4671_ABN_2_DECODER_MODE                 0x2C
	#define TMC4671_ABN_2_DECODER_PPR                  0x2D
	#define TMC4671_ABN_2_DECODER_COUNT                0x2E
	#define TMC4671_ABN_2_DECODER_COUNT_N              0x2F
	#define TMC4671_ABN_2_DECODER_PHI_M_OFFSET         0x30
	#define TMC4671_ABN_2_DECODER_PHI_M                0x31

	#define TMC4671_HALL_MODE                          0x33
	#define TMC4671_HALL_POSITION_060_000              0x34
	#define TMC4671_HALL_POSITION_180_120              0x35
	#define TMC4671_HALL_POSITION_300_240              0x36
	#define TMC4671_HALL_PHI_E_PHI_M_OFFSET            0x37
	#define TMC4671_HALL_DPHI_MAX                      0x38
	#define TMC4671_HALL_PHI_E_INTERPOLATED_PHI_E      0x39
	#define TMC4671_HALL_PHI_M                         0x3A

	#define TMC4671_AENC_DECODER_MODE                  0x3B
	#define TMC4671_AENC_DECODER_N_MASK_N_THRESHOLD    0x3C
	#define TMC4671_AENC_DECODER_PHI_A_RAW             0x3D
	#define TMC4671_AENC_DECODER_PHI_A_OFFSET          0x3E
	#define TMC4671_AENC_DECODER_PHI_A                 0x3F

	#define TMC4671_AENC_DECODER_PPR                   0x40
	#define TMC4671_AENC_DECODER_COUNT                 0x41
	#define TMC4671_AENC_DECODER_COUNT_N               0x42
	#define TMC4671_AENC_DECODER_PHI_E_PHI_M_OFFSET    0x45
	#define TMC4671_AENC_DECODER_PHI_E_PHI_M           0x46
	#define TMC4671_AENC_DECODER_POSITION              0x47

	#define TMC4671_PIDIN_TORQUE_TARGET_FLUX_TARGET    0x4A
	#define TMC4671_PIDIN_VELOCITY_TARGET              0x4B
	#define TMC4671_PIDIN_POSITION_TARGET              0x4C

	#define TMC4671_CONFIG_DATA                        0x4D
	#define TMC4671_CONFIG_ADDR                        0x4E

	/*
		Bit 15-8: VELOCITY_METER_SELECTION (0: default, 1: advanced)
		Bit 7-0: Selects the source of the velocity source for velocity measurement.
			0: PHI_E_SELECTION
			1: phi_e_ext
			2: phi_e_openloop
			3: phi_e_abn
			4: reserved
			5: phi_e_hal
			6: phi_e_aenc
			7: phi_a_aenc
			8: reserved
			9: phi_m_abn
			10: phi_m_abn_2
			11: phi_m_aenc
			12: phi_m_hal
	*/
	#define TMC4671_VELOCITY_SELECTION                 0x50
	/*
		Bit 7-0: POSITION_SELECTION
			0: phi_e selected via PHI_E_SELECTION
			1: phi_e_ext
			2: phi_e_openloop
			3: phi_e_abn
			4: reserved
			5: phi_e_hal
			6: phi_e_aenc
			7: phi_a_aenc
			8: reserved
			9: phi_m_abn
			10: phi_m_abn_2
			11: phi_m_aenc
			12: phi_m_hal
	*/
	#define TMC4671_POSITION_SELECTION                 0x51
	/*
		Bit 7-0: PHI_E_SELECTION
			0: reserved
			1: phi_e_ext
			2: phi_e_openloop
			3: phi_e_abn
			4: reserved
			5: phi_e_hal
			6: phi_e_aenc
			7: phi_a_aenc
	*/
	#define TMC4671_PHI_E_SELECTION                    0x52
	#define TMC4671_PHI_E                              0x53

	/*
        Bit 31-16: PID_FLUX_P
        Bit 15-0: PID_FLUX_I
    */
	#define TMC4671_PID_FLUX_P_FLUX_I                  0x54
	/*
        Bit 31-16: PID_TORQUE_P
        Bit 15-0: PID_TORQUE_I
    */
	#define TMC4671_PID_TORQUE_P_TORQUE_I              0x56
	/*
        Bit 31-16: PID_VELOCITY_P
        Bit 15-0: PID_VELOCITY_I
    */
	#define TMC4671_PID_VELOCITY_P_VELOCITY_I          0x58
	/*
        Bit 31-16: PID_POSITION_P
        Bit 15-0: PID_POSITION_I
    */
	#define TMC4671_PID_POSITION_P_POSITION_I          0x5A
	#define TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS  0x5C
	#define TMC4671_PIDOUT_UQ_UD_LIMITS                0x5D

	/*
		Bit 15-0: PID_TORQUE_FLUX_LIMITS, PID torque limt and PID flux limit, limits the target values coming from the target registers.
	*/
	#define TMC4671_PID_TORQUE_FLUX_LIMITS             0x5E
	/*
		Bit 15-0: PID_ACCELERATION_LIMITS
	*/
	#define TMC4671_PID_ACCELERATION_LIMIT             0x5F
	/*
		Bit 31-0: PID_VELOCITY_LIMIT
	*/
	#define TMC4671_PID_VELOCITY_LIMIT                 0x60
	#define TMC4671_PID_POSITION_LIMIT_LOW             0x61
	#define TMC4671_PID_POSITION_LIMIT_HIGH            0x62

	/*
		Bit 31: MODE_PID_TYPE (0: parallel/classic PI, 1: sequential/advanced PI)
		Bit 30-24: MODE_PID_SMPL
		Bit 7-0: MODE_MOTION
			0: stopped_mode
			1: torque_mode
			2: velocity_mode
			3: position_mode
			4: prbs_flux_mode
			5: prbs_torque_mode
			6: prbs_velocity_mode
			7: prbs_position_mode
			8: uq_ud_ext
			9: reserved
			10: AGPI_A torque_mode
			11: AGPI_A velocity_mode
			12: AGPI_A position_mode
			13: PWM_I torque_mode
			14: PWM_I velocity_mode
			15: PWM_I position_mode
	*/
	#define TMC4671_MODE_RAMP_MODE_MOTION              0x63
	#define TMC4671_PID_TORQUE_FLUX_TARGET             0x64
	#define TMC4671_PID_TORQUE_FLUX_OFFSET             0x65
	#define TMC4671_PID_VELOCITY_TARGET                0x66
	#define TMC4671_PID_VELOCITY_OFFSET                0x67
	/*
		Bit 31-0: PID_POSITION_TARGET, Target position register (for position mode).
	*/
	#define TMC4671_PID_POSITION_TARGET                0x68

	#define TMC4671_PID_TORQUE_FLUX_ACTUAL             0x69
	#define TMC4671_PID_VELOCITY_ACTUAL                0x6A
	/*
		Bit 31-0: PID_POSITION_ACTUAL, Actual multi turn position for positioning. Input position differences are accumulated. 
		Lower 16 bits display one revolution of input angle. Upper 16 bits display revolutions. 
		WRITE on PID_POSITION_ACTUAL writes same value into PID_POSITION_TARGET to avoid unwanted move.
	*/
	#define TMC4671_PID_POSITION_ACTUAL                0x6B

	#define TMC4671_PID_ERROR_DATA                     0x6C
	#define TMC4671_PID_ERROR_ADDR                     0x6D
	#define TMC4671_INTERIM_DATA                       0x6E
	#define TMC4671_INTERIM_ADDR                       0x6F

	#define TMC4671_WATCHDOG_CFG                       0x74
	#define TMC4671_ADC_VM_LIMITS                      0x75
	#define TMC4671_INPUTS_RAW                         0x76
	#define TMC4671_OUTPUTS_RAW                        0x77

	#define TMC4671_STEP_WIDTH                         0x78

	#define TMC4671_UART_BPS                           0x79
	#define TMC4671_UART_ADDRS                         0x7A

	#define TMC4671_GPIO_dsADCI_CONFIG                 0x7B

	#define TMC4671_STATUS_FLAGS                       0x7C
	#define TMC4671_STATUS_MASK                        0x7D

	// motor types
	#define TMC4671_NO_MOTOR					0
	#define TMC4671_SINGLE_PHASE_DC				1
	#define TMC4671_TWO_PHASE_STEPPER			2
	#define TMC4671_THREE_PHASE_BLDC			3

	// motion modes
	#define TMC4671_MOTION_MODE_STOPPED    		0
	#define TMC4671_MOTION_MODE_TORQUE     		1
	#define TMC4671_MOTION_MODE_VELOCITY   		2
	#define TMC4671_MOTION_MODE_POSITION   		3
	#define TMC4671_MOTION_MODE_PRBS_FLUX       4
	#define TMC4671_MOTION_MODE_PRBS_TORQUE     5
	#define TMC4671_MOTION_MODE_PRBS_VELOCITY   6
	#define TMC4671_MOTION_MODE_PRBS_POSITION   7
	#define TMC4671_MOTION_MODE_UQ_UD_EXT  		8

	// phi_e selections
	#define TMC4671_PHI_E_EXTERNAL   			1
	#define TMC4671_PHI_E_OPEN_LOOP  			2
	#define TMC4671_PHI_E_ABN        			3
	#define TMC4671_PHI_E_HALL       			5
	#define TMC4671_PHI_E_AENC      			6
	#define TMC4671_PHI_A_AENC       			7

	// velocity selection
    #define TMC4671_VELOCITY_PHI_E_SELECTION	0
	#define TMC4671_VELOCITY_PHI_E_EXT			1
	#define TMC4671_VELOCITY_PHI_E_OPENLOOP		2
	#define TMC4671_VELOCITY_PHI_E_ABN			3

	#define TMC4671_VELOCITY_PHI_E_HAL			5
	#define TMC4671_VELOCITY_PHI_E_AENC			6
	#define TMC4671_VELOCITY_PHI_A_AENC			7

	#define TMC4671_VELOCITY_PHI_M_ABN			9
	#define TMC4671_VELOCITY_PHI_M_ABN_2		10
	#define TMC4671_VELOCITY_PHI_M_AENC			11
	#define TMC4671_VELOCITY_PHI_M_HAL			12

	// position selection
	#define TMC4671_POSITION_PHI_E_SELECTION	0
	#define TMC4671_POSITION_PHI_E_EXT			1
	#define TMC4671_POSITION_PHI_E_OPENLOOP		2
	#define TMC4671_POSITION_PHI_E_ABN			3

	#define TMC4671_POSITION_PHI_E_HAL			5
	#define TMC4671_POSITION_PHI_E_AENC			6
	#define TMC4671_POSITION_PHI_A_AENC			7

	#define TMC4671_POSITION_PHI_M_ABN			9
	#define TMC4671_POSITION_PHI_M_ABN_2		10
	#define TMC4671_POSITION_PHI_M_AENC			11
	#define TMC4671_POSITION_PHI_M_HAL			12

#endif /* TMC4671_REGISTERS_H */
