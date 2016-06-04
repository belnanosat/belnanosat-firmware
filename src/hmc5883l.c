//HMC5883L I2C library for ARM STM32F103xx Microcontrollers - Main header file
//Has bit, byte and buffer I2C R/W functions
// 24/05/2012 by Harinadha Reddy Chintalapalli <harinath.ec@gmail.com>
// Changelog:
//     2012-05-24 - initial release. Thanks to Jeff Rowberg <jeff@rowberg.net> for his AVR/Arduino
//                  based development which inspired me & taken as reference to develop this.
/* ============================================================================================
 HMC5883L device I2C library code for ARM STM32F103xx is placed under the MIT license
 Copyright (c) 2012 Harinadha Reddy Chintalapalli

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
 ================================================================================================
 */

#include "hmc5883l_fileid.h"
#include "hmc5883l.h"

#include <libopencm3/stm32/i2c.h>

#include "i2c.h"

uint8_t HMC5883Lmode;

/** @defgroup HMC5883L_Library
 * @{
 */

/** Power on and prepare for general usage.
 * This will prepare the magnetometer with default settings, ready for single-
 * use mode (very low power requirements). Default settings include 8-sample
 * averaging, 15 Hz data output rate, normal measurement bias, a,d 1090 gain (in
 * terms of LSB/Gauss). Be sure to adjust any settings you need specifically
 * after initialization, especially the gain settings if you happen to be seeing
 * a lot of -4096 values (see the datasheet for mor information).
 */
void HMC5883L_Init()
{
    // write CONFIG_A register

    uint8_t tmp = (HMC5883L_AVERAGING_1 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1))
            | (HMC5883L_RATE_75 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1))
            | (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1));
    i2c_write_byte(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, tmp);

    // write CONFIG_B register
    HMC5883L_SetGain(HMC5883L_GAIN_1090);

    /* // write MODE register */
    /* HMC5883L_SetMode(HMC5883L_MODE_SINGLE); */
	i2c_write_byte(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, 0x02, 0x00);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool HMC5883L_TestConnection()
{
    uint8_t tmp[3] = { 0 };
//    i2c_read_sequence(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_ID_A, tmp, 3);
    return (tmp[0] == 'H' && tmp[1] == '4' && tmp[2] == '3') ? 1 : 0;
}
// CONFIG_A register

/** Get number of samples averaged per measurement.
 * @return Current samples averaged per measurement (0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_AVERAGING_8
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
uint8_t HMC5883L_GetSampleAveraging()
{
	return i2c_read_bits(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A,
                         HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH);
}

/** Set number of samples averaged per measurement.
 * @param averaging New samples averaged per measurement setting(0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
void HMC5883L_SetSampleAveraging(uint8_t averaging)
{
	i2c_write_bits(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A,
	               HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, averaging);
}

/** \code
 * Get data output rate value.
 * The Table below shows all selectable output rates in continuous measurement
 * mode. All three channels shall be measured within a given output rate. Other
 * output rates with maximum rate of 160 Hz can be achieved by monitoring DRDY
 * interrupt pin in single measurement mode.
 *
 * Value | Typical Data Output Rate (Hz)
 * ------+------------------------------
 * 0     | 0.75
 * 1     | 1.5
 * 2     | 3
 * 3     | 7.5
 * 4     | 15 (Default)
 * 5     | 30
 * 6     | 75
 * 7     | Not used
 * \endcode
 * @return Current rate of data output to registers
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
uint8_t HMC5883L_GetDataRate()
{
	return i2c_read_bits(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A,
                         HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH);
}

/** Set data output rate value.
 * @param rate Rate of data output to registers
 * @see HMC5883L_SetDataRate()
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
void HMC5883L_SetDataRate(uint8_t rate)
{
	i2c_write_bits(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A,
	               HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, rate);
}

/** Get measurement bias value.
 * @return Current bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
uint8_t HMC5883L_GetMeasurementBias()
{
	return i2c_read_bits(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A,
	                     HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH);
}

/** Set measurement bias value.
 * @param bias New bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
void HMC5883L_SetMeasurementBias(uint8_t bias)
{
	i2c_write_bits(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A,
	               HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, bias);
}

// CONFIG_B register

/** \code
 * Get magnetic field gain value.
 * The table below shows nominal gain settings. Use the "Gain" column to convert
 * counts to Gauss. Choose a lower gain value (higher GN#) when total field
 * strength causes overflow in one of the data output registers (saturation).
 * The data output range for all settings is 0xF800-0x07FF (-2048 - 2047).
 *
 * Value | Field Range | Gain (LSB/Gauss)
 * ------+-------------+-----------------
 * 0     | +/- 0.88 Ga | 1370
 * 1     | +/- 1.3 Ga  | 1090 (Default)
 * 2     | +/- 1.9 Ga  | 820
 * 3     | +/- 2.5 Ga  | 660
 * 4     | +/- 4.0 Ga  | 440
 * 5     | +/- 4.7 Ga  | 390
 * 6     | +/- 5.6 Ga  | 330
 * 7     | +/- 8.1 Ga  | 230
 * \endcode
 * @return Current magnetic field gain value
 * @see HMC5883L_GAIN_1090
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
uint8_t HMC5883L_GetGain()
{
    return i2c_read_bits(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_B,
                         HMC5883L_CRB_GAIN_BIT, HMC5883L_CRB_GAIN_LENGTH);
}

/** Set magnetic field gain value.
 * @param gain New magnetic field gain value
 * @see HMC5883L_GetGain()
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
void HMC5883L_SetGain(uint8_t gain)
{
    // use this method to guarantee that bits 4-0 are set to zero, which is a
    // requirement specified in the datasheet;
    uint8_t tmp = gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1);
    i2c_write_byte(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS,
                   HMC5883L_RA_CONFIG_B, tmp);
}

// MODE register

/** Get measurement mode.
 * In continuous-measurement mode, the device continuously performs measurements
 * and places the result in the data register. RDY goes high when new data is
 * placed in all three registers. After a power-on or a write to the mode or
 * configuration register, the first measurement set is available from all three
 * data output registers after a period of 2/fDO and subsequent measurements are
 * available at a frequency of fDO, where fDO is the frequency of data output.
 *
 * When single-measurement mode (default) is selected, device performs a single
 * measurement, sets RDY high and returned to idle mode. Mode register returns
 * to idle mode bit values. The measurement remains in the data output register
 * and RDY remains high until the data output register is read or another
 * measurement is performed.
 *
 * @return Current measurement mode
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
uint8_t HMC5883L_GetMode()
{
    return i2c_read_bits(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS,
                         HMC5883L_RA_MODE, HMC5883L_MODEREG_BIT,
                         HMC5883L_MODEREG_LENGTH);
}

/** Set measurement mode.
 * @param newMode New measurement mode
 * @see HMC5883L_GetMode()
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
void HMC5883L_SetMode(uint8_t newMode)
{
    // use this method to guarantee that bits 7-2 are set to zero, which is a
    // requirement specified in the datasheet;
	uint8_t tmp = 0;//HMC5883Lmode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1);
    i2c_write_byte(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, tmp);
    HMC5883Lmode = newMode; // track to tell if we have to clear bit 7 after a read
}

// DATA* registers

/** Get 3-axis heading measurements.
 * In the event the ADC reading overflows or underflows for the given channel,
 * or if there is a math overflow during the bias measurement, this data
 * register will contain the value -4096. This register value will clear when
 * after the next valid measurement is made. Note that this method automatically
 * clears the appropriate bit in the MODE register if Single mode is active.
 * @param x 16-bit signed integer container for X,Y,Z-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
void HMC5883L_GetHeading(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t buffer[6];

	uint32_t reg32 __attribute__((unused));
	uint32_t i2c = HMC5883L_I2C;
	int i;


//	i2c_read_sequence(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, buffer, 6);

	while (I2C_SR2(i2c) & I2C_SR2_BUSY);
	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	         & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	i2c_send_7bit_address(i2c, HMC5883L_ADDRESS, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	i2c_send_data(i2c, 0x03);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
	i2c_send_stop(i2c);

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	         & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, HMC5883L_ADDRESS, I2C_READ);
	I2C_CR1(i2c) |= I2C_CR1_ACK;
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);

	for (i = 0; i + 1 < 6; ++i) {
		while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
		buffer[i] = I2C_DR(i2c);
	}


	I2C_CR1(i2c) &= ~I2C_CR1_ACK;
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	buffer[5] = I2C_DR(i2c);

	i2c_send_stop(i2c);

	*x = (((int16_t)buffer[0]) << 8) | buffer[1];
	*y = (((int16_t)buffer[4]) << 8) | buffer[5];
	*z = (((int16_t)buffer[2]) << 8) | buffer[3];

	/* while (I2C_SR2(i2c) & I2C_SR2_BUSY); */
	/* i2c_send_start(i2c); */
	/* while (!((I2C_SR1(i2c) & I2C_SR1_SB) */
	/*          & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)))); */
	/* i2c_send_7bit_address(i2c, HMC5883L_ADDRESS, I2C_WRITE); */
	/* while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)); */
	/* reg32 = I2C_SR2(i2c); */

	/* i2c_send_data(i2c, 0x06); */
	/* while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE))); */

	/* i2c_send_start(i2c); */
	/* while (!((I2C_SR1(i2c) & I2C_SR1_SB) */
	/*          & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)))); */
	/* i2c_send_7bit_address(i2c, HMC5883L_ADDRESS, I2C_READ); */
	/* while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)); */
	/* reg32 = I2C_SR2(i2c); */

	/* I2C_CR1(i2c) |= I2C_CR1_ACK; */
	/* while (!(I2C_SR1(i2c) & I2C_SR1_RxNE)); */
	/* *x = (uint16_t)(I2C_DR(i2c) << 8); */
	/* I2C_CR1(i2c) |= I2C_CR1_ACK; */
	/* while (!(I2C_SR1(i2c) & I2C_SR1_RxNE)); */
	/* *x |= I2C_DR(i2c); */

	/* while (!(I2C_SR1(i2c) & I2C_SR1_RxNE)); */
	/* *y = (uint16_t)(I2C_DR(i2c) << 8); */
	/* while (!(I2C_SR1(i2c) & I2C_SR1_RxNE)); */
	/* *y |= I2C_DR(i2c); */

	/* while (!(I2C_SR1(i2c) & I2C_SR1_RxNE)); */
	/* *z = (uint16_t)(I2C_DR(i2c) << 8); */
	/* while (!(I2C_SR1(i2c) & I2C_SR1_RxNE)); */
	/* *z |= I2C_DR(i2c); */
	/* i2c_send_stop(i2c); */
	/* *x = i2c_read_word(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H); */
	/* *y = i2c_read_word(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAY_H); */
	/* *z = i2c_read_word(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAZ_H); */

/* 	uint8_t tmp = HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1); */

/* //	if (HMC5883Lmode == HMC5883L_MODE_SINGLE) */
/* 		i2c_write_byte(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, tmp); */
}

// STATUS register

/** Get data output register lock status.
 * This bit is set when this some but not all for of the six data output
 * registers have been read. When this bit is set, the six data output registers
 * are locked and any new data will not be placed in these register until one of
 * three conditions are met: one, all six bytes have been read or the mode
 * changed, two, the mode is changed, or three, the measurement configuration is
 * changed.
 * @return Data output register lock status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_LOCK_BIT
 */
int HMC5883L_GetLockStatus()
{
	return i2c_read_bit(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS,
	                    HMC5883L_RA_STATUS, HMC5883L_STATUS_LOCK_BIT);
}

/** Get data ready status.
 * This bit is set when data is written to all six data registers, and cleared
 * when the device initiates a write to the data output registers and after one
 * or more of the data output registers are written to. When RDY bit is clear it
 * shall remain cleared for 250 us. DRDY pin can be used as an alternative to
 * the status register for monitoring the device for measurement data.
 * @return Data ready status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_READY_BIT
 */
int HMC5883L_GetReadyStatus()
{
	return i2c_read_bit(HMC5883L_I2C, HMC5883L_DEFAULT_ADDRESS,
	                    HMC5883L_RA_STATUS, HMC5883L_STATUS_READY_BIT);
}

/**
 * @}
 *//* end of group HMC5883L_Library */
