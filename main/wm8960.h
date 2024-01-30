#ifndef ZEPHYR_DRIVERS_SENSOR_WM8960_WM8960_H_
#define ZEPHYR_DRIVERS_SENSOR_WM8960_WM8960_H_

#include "esp_bit_defs.h"

/* Register definitions */

#define WM8960_R0_LEFT_INPUT_VOLUME_REG     0x00
#define WM8960_R1_RIGHT_INPUT_VOLUME_REG    0x01
#define WM8960_R2_LOUT1_VOLUME_REG          0x02
#define WM8960_R3_ROUT1_VOLUME_REG          0x03
#define WM8960_R4_CLOCKING_1_REG            0x04
#define WM8960_R5_ADC_AND_DAC_CTRL1_REG     0x05
#define WM8960_R6_ADC_AND_DAC_CTRL2_REG     0x06
#define WM8960_R7_AUDIO_INTERFACE_REG       0x07
#define WM8960_R8_CLOCKING_2_REG            0x08
#define WM8960_R9_AUDIO_INTERFACE_REG       0x09
#define WM8960_R10_LEFT_DAC_VOLUME_REG      0x0A
#define WM8960_R11_RIGHT_DAC_VOLUME_REG     0x0B
#define WM8960_R15_RESET_REG                0x0F
#define WM8960_R16_3D_CONTROL_REG           0x10
#define WM8960_R17_ALC1_REG                 0x11
#define WM8960_R18_ALC2_REG                 0x12
#define WM8960_R19_ALC3_REG                 0x13
#define WM8960_R20_NOISE_GATE_REG           0x14
#define WM8960_R21_LEFT_ADC_VOLUME_REG      0x15
#define WM8960_R22_RIGHT_ADC_VOLUME_REG     0x16
#define WM8960_R23_ADDITIONAL_CONTROL_1_REG 0x17
#define WM8960_R24_ADDITIONAL_CONTROL_2_REG 0x18
#define WM8960_R25_PWR_MGMT_1_REG           0x19
#define WM8960_R26_PWR_MGMT_2_REG           0x1A
#define WM8960_R27_ADDITIONAL_CONTROL_3_REG 0x1B
#define WM8960_R28_ANTI_POP_1_REG           0x1C
#define WM8960_R29_ANTI_POP_2_REG           0x1D
#define WM8960_R32_ADCL_SIGNAL_PATH_REG     0x20
#define WM8960_R33_ADCR_SIGNAL_PATH_REG     0x21
#define WM8960_R34_LEFT_OUT_MIX_1_REG       0x22
#define WM8960_R37_RIGHT_OUT_MIX_2_REG      0x25
#define WM8960_R38_MONO_OUT_MIX_1_REG       0x26
#define WM8960_R39_MONO_OUT_MIX_2_REG       0x27
#define WM8960_R40_LOUT2_VOLUME_REG         0x28
#define WM8960_R41_ROUT2_VOLUME_REG         0x29
#define WM8960_R42_MONOOUT_VOLUME_REG       0x2A
#define WM8960_R43_INPUT_BOOST_MIXER_1_REG  0x2B
#define WM8960_R44_INPUT_BOOST_MIXER_2_REG  0x2C
#define WM8960_R45_BYPASS_1_REG             0x2D
#define WM8960_R46_BYPASS_2_REG             0x2E
#define WM8960_R47_PWR_MGMT_3_REG           0x2F
#define WM8960_R48_ADDITIONAL_CONTROL_4_REG 0x30
#define WM8960_R49_CLASS_D_CONTROL_1_REG    0x31
#define WM8960_R51_CLASS_D_CONTROL_3_REG    0x33
#define WM8960_R52_PPL_N_REG                0x34
#define WM8960_R53_PPL_K_1_REG              0x35
#define WM8960_R54_PPL_K_2_REG              0x36
#define WM8960_R55_PPL_K_3_REG              0x37

/* Left Input Volume  */
#define WM8960_R0_IPVU BIT(8)
#define WM8960_R0_LINMUTE BIT(7)
#define WM8960_R0_LIZC BIT(6)
#define WM8960_R0_LINVOL(x) (((x) & 0x3F) << 0)

/* Right Input Volume */
#define WM8960_R1_IPVU BIT(8)
#define WM8960_R1_LINMUTE BIT(7)
#define WM8960_R1_LIZC BIT(6)
#define WM8960_R1_LINVOL(x) (((x) & 0b111111) << 0)

/* LOUT1 Volume */
#define WM8960_R2_OUT1VU BIT(8)
#define WM8960_R2_LO1ZC BIT(7)
#define WM8960_R2_LOUT1VOL(x) (((x) & 0b1111111) << 0)

/* ROUT1 Volume */
#define WM8960_R3_OUT1VU BIT(8)
#define WM8960_R3_LO1ZC BIT(7)
#define WM8960_R3_LOUT1VOL(x) (((x) & 0b1111111) << 0)

/* Clocking */
#define WM8960_R4_ADCDIV(x) (((x) & 0b111) << 6)
#define WM8960_R4_DACDIV(x) (((x) & 0b111) << 3)
#define WM8960_R4_SYSCLKDIV(x) (((x) & 0b11) << 1)
#define WM8960_R4_CLKSEL BIT(0)

/* ADC and DAC Control (1) */
#define WM8960_R5_RESERVED8 BIT(8)
#define WM8960_R5_DACDIV2 BIT(7)
#define WM8960_R5_ADCPOL(x) (((x) & 0b11) << 5)
#define WM8960_R5_RESERVED4 BIT(4)
#define WM8960_R5_DACMU BIT(3)
#define WM8960_R5_DEEMPH(x) (((x) & 0b11) << 1)
#define WM8960_R5_ADCHPD BIT(0)

/* ADC and DAC Control (2) */
#define WM8960_R6_RESERVED7(x) (((x) & 0b11) << 7)
#define WM8960_R6_DACPOL(x) (((x) & 0b11) << 5)
#define WM8960_R6_RESERVED4 BIT(4)
#define WM8960_R6_DACSMM BIT(3)
#define WM8960_R6_DACMR BIT(2)
#define WM8960_R6_DACSLOPE BIT(1)
#define WM8960_R6_RESERVED0 BIT(0)

/* Audio Interface */
#define WM8960_R7_ALRSWAP BIT(8)
#define WM8960_R7_BCLKINV BIT(7)
#define WM8960_R7_MS BIT(6)
#define WM8960_R7_DLRSWAP BIT(5)
#define WM8960_R7_LRP BIT(4)
#define WM8960_R7_WL(x) (((x) & 0b11) << 2)
#define WM8960_R7_FORMAT(x) (((x) & 0b11) << 0)

/* Clocking (2) */
#define WM8960_R8_DCLKDIV(x) (((x) & 0b111) << 6)
#define WM8960_R8_RESERVED4(x) (((x) & 0b11) << 4)
#define WM8960_R8_BCLKDIV(x) (((x) & 0b1111) << 0)

/* Audio Interface */
#define WM8960_R9_RESERVED7(x) (((x) & 0b11) << 7)
#define WM8960_R9_ALRCGPIO BIT(6)
#define WM8960_R9_WL8 BIT(5)
#define WM8960_R9_DACCOMP(x) (((x) & 0b11) << 3)
#define WM8960_R9_ADCCOMP(x) (((x) & 0b11) << 1)
#define WM8960_R9_LOOPBACK BIT(0)

/* Left DAC Volume */
#define WM8960_R10_DACVU BIT(8)
#define WM8960_R10_LDACVOL(x) (((x) & 0b11111111) << 0)

/* Right DAC Volume */
#define WM8960_R11_DACVU BIT(8)
#define WM8960_R11_RDACVOL(x) (((x) & 0b11111111) << 0)

/* Reset */
#define WM8960_R15_RESET(x) (((x) & 0b111111111) << 0)

/* 3D Control */
#define WM8960_R16_RESERVED8 BIT(8)
#define WM8960_R16_RESERVED7 BIT(7)
#define WM8960_R16_3DUC BIT(6)
#define WM8960_R16_3DLC BIT(5)
#define WM8960_R16_3DDEPTH(x) (((x) & 0b1111) << 1)
#define WM8960_R16_3DEN BIT(0)

/* ALC (1) */
#define WM8960_R17_ALCSEL(x) (((x) & 0b11) << 7)
#define WM8960_R17_MAXGAIN(x) (((x) & 0b1111) << 4)
#define WM8960_R17_ALCL(x) (((x) & 0b1111) << 0)

/* ALC (2) */
#define WM8960_R18_RESERVED8 BIT(8)
#define WM8960_R18_RESERVED7 BIT(7)
#define WM8960_R18_MINGAIN(x) (((x) & 0b111) << 4)
#define WM8960_R18_HLD(x) (((x) & 0b1111) << 0)

/* ALC (3) */
#define WM8960_R19_ALCMODE BIT(8)
#define WM8960_R19_DCY(x) (((x) & 0b1111) << 4)
#define WM8960_R19_ATK(x) (((x) & 0b1111) << 0)

/* Noise Gate */
#define WM8960_R20_RESERVED8 BIT(8)
#define WM8960_R20_NGTH(x) (((x) & 0b11111) << 3)
#define WM8960_R20_RESERVED1(x) (((x) & 0b11) << 1)
#define WM8960_R20_NGAT BIT(0)

/* Left ADC Volume */
#define WM8960_R21_ADCVU BIT(8)
#define WM8960_R21_LADCVOL(x) (((x) & 0b11111111) << 0)

/* Right ADC Volume */
#define WM8960_R22_ADCVU BIT(8)
#define WM8960_R22_RADCVOL(x) (((x) & 0b11111111) << 0)

/* Additional Control (1) */
#define WM8960_R23_TSDEN BIT(8)
#define WM8960_R23_VSEL(x) (((x) & 0b11) << 6)
#define WM8960_R23_RESERVED5 BIT(5)
#define WM8960_R23_DMONOMIX BIT(4)
#define WM8960_R23_DATSEL(x) (((x) & 0b11) << 2)
#define WM8960_R23_TOCLKSEL BIT(1)
#define WM8960_R23_TOEN BIT(0)

/* Additional Control (2) */
#define WM8960_R24_RESERVED7 (x) (((x) & 0b11) << 7)
#define WM8960_R24_HPSWEN BIT(6)
#define WM8960_R24_HPSWPOL BIT(5)
#define WM8960_R24_RESERVED4 BIT(4)
#define WM8960_R24_TRIS BIT(3)
#define WM8960_R24_LCRM BIT(2)
#define WM8960_R24_RESERVED0(x) (((x) & 0b11) << 0)

/* Power Mgmt (1) */
#define WM8960_R25_VMIDSEL(x) (((x) & 0b11) << 7)
#define WM8960_R25_VREF BIT(6)
#define WM8960_R25_AINL BIT(5)
#define WM8960_R25_AINR BIT(4)
#define WM8960_R25_ADCL BIT(3)
#define WM8960_R25_ADCR BIT(2)
#define WM8960_R25_MICB BIT(1)
#define WM8960_R25_DIGENB BIT(0)

/* Power Mgmt (2) */
#define WM8960_R26_DACL BIT(8)
#define WM8960_R26_DACR BIT(7)
#define WM8960_R26_LOUT1 BIT(6)
#define WM8960_R26_ROUT1 BIT(5)
#define WM8960_R26_SPKL BIT(4)
#define WM8960_R26_SPKR BIT(3)
#define WM8960_R26_RESERVED2 BIT(2)
#define WM8960_R26_OUT3 BIT(1)
#define WM8960_R26_PLL_EN BIT(0)

/* Additional Control (3) */
#define WM8960_R27_RESERVED7(x) (((x) & 0b11) << 7)
#define WM8960_R27_VROI BIT(6)
#define WM8960_R27_RESERVED5 BIT(5)
#define WM8960_R27_RESERVED4 BIT(4)
#define WM8960_R27_OUT3CAP BIT(3)
#define WM8960_R27_ADC_ALC_SR(x) (((x) & 0b111) << 0)

/* Anti-Pop 1 */
#define WM8960_R28_RESERVED8 BIT(8)
#define WM8960_R28_POBCTRL BIT(7)
#define WM8960_R28_RESERVED5(x) (((x) & 0b11) << 5)
#define WM8960_R28_BUFDCOPEN BIT(4)
#define WM8960_R28_BUFIOEN BIT(3)
#define WM8960_R28_SOFT_ST BIT(2)
#define WM8960_R28_RESERVED1 BIT(1)
#define WM8960_R28_HPSTBY BIT(0)

/* Anti-Pop 2 */
#define WM8960_R29_RESERVED7(x) (((x) & 0b11) << 7)
#define WM8960_R29_DISOP BIT(6)
#define WM8960_R29_DRES(x) (((x) & 0b11) << 4)
#define WM8960_R29_RESERVED0(x) (((x) & 0b1111) << 0)

/* ADCL Signal Path */
#define WM8960_R32_LMN1 BIT(8)
#define WM8960_R32_LMP3 BIT(7)
#define WM8960_R32_LMP2 BIT(6)
#define WM8960_R32_LMICBOOST(x) (((x) & 0b11) << 4)
#define WM8960_R32_LMIC2B BIT(3)
#define WM8960_R32_RESERVED0(x) (((x) & 0b111) << 0)

/* ADCR Signal Path */
#define WM8960_R33_RMN1 BIT(8)
#define WM8960_R33_RMP3 BIT(7)
#define WM8960_R33_RMP2 BIT(6)
#define WM8960_R33_RMICBOOST(x) (((x) & 0b11) << 4)
#define WM8960_R33_RMIC2B BIT(3)
#define WM8960_R33_RESERVED0(x) (((x) & 0b111) << 0)

/* Left Out Mix */
#define WM8960_R34_LD2LO BIT(8)
#define WM8960_R34_LI2LO BIT(7)
#define WM8960_R34_LI2LOVOL(x) (((x) & 0b111) << 4)
#define WM8960_R34_RESERVED0 BIT(0)

/* Right Out Mix */
#define WM8960_R37_RD2RO BIT(8)
#define WM8960_R37_RI2RO BIT(7)
#define WM8960_R37_RI2ROVOL(x) (((x) & 0b111) << 4)
#define WM8960_R37_RESERVED0 BIT(0)

/* Mono Out Mix (1) */
#define WM8960_R38_RESERVED8 BIT(8)
#define WM8960_R38_L2MO BIT(7)
#define WM8960_R38_RESERVED0(x) (((x) & 0b1111111) << 0)

/* Mono Out Mix (2) */
#define WM8960_R39_RESERVED8 BIT(8)
#define WM8960_R39_R2MO BIT(7)
#define WM8960_R39_(x) (((x) & 0b1111111) << 0)

/* Left Speaker Volume */
#define WM8960_R40_SPKVU BIT(8)
#define WM8960_R40_SPKLZC BIT(7)
#define WM8960_R40_SPKLVOL(x) (((x) & 0b1111111) << 0)

/* Right Speaker Volume */
#define WM8960_R41_SPKVU BIT(8)
#define WM8960_R41_SPKRZC BIT(7)
#define WM8960_R41_SPKRVOL(x) (((x) & 0b1111111) << 0)

/* OUT3 Volume */
#define WM8960_R42_RESERVED7(x) (((x) & 0b11) << 7)
#define WM8960_R42_MOUTVOL BIT(6)
#define WM8960_R42_(x) (((x) & 0b111111) << 0)

/* Left Input Boost Mixer */
#define WM8960_R43_RESERVED7(x) (((x) & 0b11) << 7)
#define WM8960_R43_LIN3BOOST(x) (((x) & 0b111) << 4)
#define WM8960_R43_LIN2BOOST(x) (((x) & 0b111) << 1)
#define WM8960_R43_RESERVED0 BIT(0)

/* Right Input Boost Mixer */
#define WM8960_R44_RESERVED7(x) (((x) & 0b11) << 7)
#define WM8960_R44_RIN3BOOST(x) (((x) & 0b111) << 4)
#define WM8960_R44_RIN2BOOST(x) (((x) & 0b111) << 1)
#define WM8960_R44_RESERVED0 BIT(0)

/* Left Bypass */
#define WM8960_R45_RESERVED8 BIT(8)
#define WM8960_R45_LB2LO BIT(7)
#define WM8960_R45_LB2LOVOL(x) (((x) & 0b111) << 4)
#define WM8960_R45_RESERVED0(x) (((x) & 0b1111) << 0)

/* Right Bypass */
#define WM8960_R46_RESERVED8 BIT(8)
#define WM8960_R46_RB2RO BIT(7)
#define WM8960_R46_RB2ROVOL(x) (((x) & 0b111) << 4)
#define WM8960_R46_RESERVED0(x) (((x) & 0b1111) << 0)

/* Power Mgmt (3) */
#define WM8960_R47_RESERVED6(x) (((x) & 0b111) << 6)
#define WM8960_R47_LMIC BIT(5)
#define WM8960_R47_RMIC BIT(4)
#define WM8960_R47_LOMIX BIT(3)
#define WM8960_R47_ROMIX BIT(3)
#define WM8960_R47_RESERVED0(x) (((x) & 0b11) << 0)

/* Additional Control (4) */
#define WM8960_R48_RESERVED8 BIT(8)
#define WM8960_R48_GPIOPOL BIT(7)
#define WM8960_R48_GPIOSEL(x) (((x) & 0b111) << 4)
#define WM8960_R48_GPSEL(x) (((x) & 0b11) << 2)
#define WM8960_R48_TSENSEN BIT(1)
#define WM8960_R48_MBSEL BIT(0)

/* Class D Control (1) */
#define WM8960_R49_RESERVED8 BIT(8)
#define WM8960_R49_SPK_OP_EN(x) (((x) & 0b11) << 6)
#define WM8960_R49_RESERVED0(x) (((x) & 0b111111) << 0)

/* Class D Control (2) */
#define WM8960_R51_RESERVED6(x) (((x) & 0b111) << 6)
#define WM8960_R51_DCGAIN(x) (((x) & 0b111) << 3)
#define WM8960_R51_ACGAIN(x) (((x) & 0b111) << 0)

/* PLL (1) */
#define WM8960_R52_OPCLKDIV(x) (((x) & 0b111) << 6)
#define WM8960_R52_SDM BIT(5)
#define WM8960_R52_PLLPRESCALE BIT(4)
#define WM8960_R52_PLLN(x) (((x) & 0b1111) << 0)

/* PLL (2) */
#define WM8960_R53_RESERVED8 BIT(8)
#define WM8960_R53_PLLK_23_16(x) (((x) & 0b11111111) << 0)

/* PLL (3) */
#define WM8960_R54_RESERVED8 BIT(8)
#define WM8960_R54_PLLK_15_8(x) (((x) & 0b11111111) << 0)

/* PLL (4) */
#define WM8960_R55_RESERVED8 BIT(8)
#define WM8960_R55_PLLK_7_0(x) (((x) & 0b11111111) << 0)

#endif /* ZEPHYR_DRIVERS_SENSOR_WM8960_WM8960_H_ */