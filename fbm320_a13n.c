/*
 * Copyright (C) 2015 Formosa Measurement Technology Inc. Ltd. All rights
 * reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

/* Driver description
 *
 * File Name    :
 * Authors      : conrad@fmti.com.tw
 * Version      : 1.0.0
 * Date         : 2018/4/23
 * Description  : FBM320-A13N pressure sensor API for MCU of ARM_M0 core
 *
 */

/* Revised history
 * 1.0.0: first release
 *
 */

/* Porting guideline
 *
 * 1. Please tailor the following three function according to your platform. These functions
 *    are depending on soc flatform.
 * static uint8_t fbm320_i2c_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data)
 * static uint8_t fbm320_i2c_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data)
 * static void fbm320_us_delay(uint32_t us)
 *
 * 2. This driver need a timer interrupt per millisecond for updating barometer data.
 * 3. Setting slave i2c address.
 *    { I2C 7bit address setting for fbm320 }
 * 	  If SDO pin is pulled low, I2C address is 6c.
 * 	  If SDO pin is pulled high, I2C address is 6d.
 * 	  #define FBM320_I2C_SLAVE_ADDR 0x6c
 * 4. Setting oversampling rate for adc conversion
 *    Define the oversampling rate setting of fbm320.
 *    Range of setting:
 *    {osr_1024, osr_2048, osr_4096, osr_8192, osr_16384}
 *    #define OVERSAMPLING_RATE_DEFAULT  osr_8192
 *
 */

#include "fbm320_a13n.h"

volatile uint32_t TMR0_Ticks;
volatile uint32_t fbm320_update_rdy;

static void fbm320_us_delay(uint32_t us);
#ifdef SPI
static uint8_t fbm320_spi_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data);
static uint8_t fbm320_spi_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data);
#else
static uint8_t fbm320_i2c_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data);
static uint8_t fbm320_i2c_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data);
#endif
static int32_t fbm320_startMeasure_temp(struct fbm320_data *barom);
static int32_t fbm320_get_raw_temperature(struct fbm320_data *barom);
static int32_t fbm320_startMeasure_press(struct fbm320_data *barom);
static int32_t fbm320_get_raw_pressure(struct fbm320_data *barom);
static int32_t fbm320_read_store_otp_data(struct fbm320_data *barom);
static int32_t fbm320_version_identification(struct fbm320_data *barom);
static int32_t fbm320_set_oversampling_rate(struct fbm320_data *barom
        , enum fbm320_osr osr_setting);
static int32_t fbm320_chipid_check(struct fbm320_data *barom);
static int32_t fbm320_calculation(struct fbm320_data *barom);
/**
 * { pointer of fbm320 data }
 */
static struct fbm320_data fbm320_barom;
struct fbm320_data *barom = &fbm320_barom;

#ifdef SPI
static uint8_t fbm320_spi_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data)
{
	/* This is just an example. This function have to
	   be implemented according to your platform. */
	uint8_t cmd, i;

	switch (cnt) {
	case 1:
		cmd = FBM320_SPI_WRITE | FBM320_SPI_1BYTE;
		break;
	case 2:
		cmd = FBM320_SPI_WRITE | FBM320_SPI_2BYTE;
		break;
	case 3:
		cmd = FBM320_SPI_WRITE | FBM320_SPI_3BYTE;
		break;
	default:
		cmd = FBM320_SPI_WRITE | FBM320_SPI_4BYTE;
	}
	SPI_SET_SS0_LOW(SPI0);
	/* Write to TX register */
	SPI_WRITE_TX0(SPI0, cmd);
	/* Trigger SPI data transfer */
	SPI_TRIGGER(SPI0);
	while (SPI_IS_BUSY(SPI0));
	SPI_WRITE_TX0(SPI0, (reg_addr + (cnt - 1)));
	SPI_TRIGGER(SPI0);
	while (SPI_IS_BUSY(SPI0));
	for (i = 0; i < cnt; i++) {
		SPI_WRITE_TX0(SPI0, *(reg_data + i));
		SPI_TRIGGER(SPI0);
		/* Check SPI0 busy status */
		while (SPI_IS_BUSY(SPI0));
	}
	SPI_SET_SS0_HIGH(SPI0);

	return 0;
}
static uint8_t fbm320_spi_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data)
{
	/* This is just an example. This function have to
	   be implemented according to your platform. */

	int8_t i;
	uint8_t cmd;
	uint32_t tmp;

	switch (cnt) {
	case 1:
		cmd = FBM320_SPI_READ | FBM320_SPI_1BYTE;
		break;
	case 2:
		cmd = FBM320_SPI_READ | FBM320_SPI_2BYTE;
		break;
	case 3:
		cmd = FBM320_SPI_READ | FBM320_SPI_3BYTE;
		break;
	default:
		cmd = FBM320_SPI_READ | FBM320_SPI_4BYTE;
	}
	SPI_SET_SS0_LOW(SPI0);
	/* Write to TX register */
	SPI_WRITE_TX0(SPI0, cmd);
	SPI_TRIGGER(SPI0);
	while (SPI_IS_BUSY(SPI0));
	SPI_WRITE_TX0(SPI0, reg_addr + (cnt - 1));
	SPI_TRIGGER(SPI0);
	while (SPI_IS_BUSY(SPI0));
	for (i = (cnt - 1); i >= 0; i--) {
		SPI_WRITE_TX0(SPI0, 0x00);//dummy clock
		SPI_TRIGGER(SPI0);
		while (SPI_IS_BUSY(SPI0));
		tmp = SPI_READ_RX0(SPI0);
//			printf("SPI read: %#x\n\r", tmp);
		*(reg_data + i) = tmp;
	}
	SPI_SET_SS0_HIGH(SPI0);

	return 0;
}
#endif

#ifdef I2C
static uint8_t fbm320_i2c_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data)
{
	/* This is just an example. This function have to
	   be implemented according to your platform. */
	uint8_t status;
	uint32_t cnt_write;
	cnt_write = I2C_WriteMultiBytesOneReg(I2C0, FBM320_I2C_SLAVE_ADDR, reg_addr \
	                                      , reg_data, cnt);
	status = (cnt_write > 0) ?  0 : 1;
	return status;
}
static uint8_t fbm320_i2c_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data)
{
	/* This is just an example. This function have to
	   be implemented according to your platform. */
	uint8_t status;
	uint32_t cnt_read;

	cnt_read = I2C_ReadMultiBytesOneReg(I2C0, FBM320_I2C_SLAVE_ADDR\
	                                    , reg_addr, reg_data, cnt);
	status = (cnt_read > 0) ?  0 : 1;
	return status;
}
#endif
/**
 * @brief      { API for fbm320 delay }
 *
 * @param[in]  us    { delay time in microseconds }
 */
static void fbm320_us_delay(uint32_t us)
{
	/* This is just an example. This function have to
	   be implemented according to your platform. */
	CLK_SysTickDelay(us);
}
/**
 * @brief      { API for assigning function pointers, as bus read/write
 *               and delay. }
 *
 * @return     { 0, succeeded
 *              -1, failed }
 */
int8_t fbm320_init(void)
{
	int32_t err;
	uint8_t data_buf;

	fbm320_barom.delay_usec = fbm320_us_delay;
	/* The minimum start up time of fbm320 is 15ms */
	barom->delay_usec(1000 * 15);

#ifdef SPI
	fbm320_barom.bus_write = fbm320_spi_writeblock;
	fbm320_barom.bus_read = fbm320_spi_readblock;
	/* The default of SPI is in 3 wires mode after power on reset. If 4 wires SPI
    mode is preffered, the following statements will be needed. */
    #define SPI_4_WIRES_MODE
	#ifdef SPI_4_WIRES_MODE
		/* Set SPI bus as 4 wires mode */
		data_buf = FBM320_SPI_CTRL_REG_SDO_ACTIVE_EN;
		barom->bus_write(FBM320_SPI_CTRL_REG, sizeof(uint8_t), &data_buf);
	#endif
#else
	fbm320_barom.bus_write = fbm320_i2c_writeblock;
	fbm320_barom.bus_read = fbm320_i2c_readblock;
#endif
	
	err = fbm320_chipid_check(barom);
	if (err) {
		err = -1;
		goto err_chip_id_chk;
	} else {
#ifdef DEBUG_FBM320
		printf("%s:fbm320_chipid_check() passed!\n", __func__);
#endif
	}

	err = fbm320_version_identification(barom);
	if (err) {
		err = -2;
		goto err_version_identification;
	} else {
#ifdef DEBUG_FBM320
		printf("%s:fbm320_version_identification() passed!\n", __func__);
#endif
	}

	err = fbm320_read_store_otp_data(barom);
	if (err) {
		err = -3;
		goto err_read_otp;
	} else {
#ifdef DEBUG_FBM320
		printf("%s:fbm320_read_store_otp_data() passed!\n", __func__);
#endif
	}
	err = 0;

	fbm320_set_oversampling_rate(barom, OVERSAMPLING_RATE_DEFAULT);
	/* Setting the P_CONFIG_REG_GAIN */
#define P_CONFIG_REG_GAIN_SETTING FBM320_P_CONFIG_REG_GAIN_X32
	barom->bus_read(FBM320_P_CONFIG_REG, sizeof(uint8_t), &data_buf);
	data_buf &= ~(FBM320_P_CONFIG_REG_GAIN_MAK);
	data_buf |= P_CONFIG_REG_GAIN_SETTING;
	barom->bus_write(FBM320_P_CONFIG_REG, sizeof(uint8_t), &data_buf);
#ifdef DEBUG_FBM320
	printf("%s:Setting of FBM320_P_CONFIG_REG_GAIN: %#x\n", __func__, P_CONFIG_REG_GAIN_SETTING);
#endif

#ifdef DEBUG_FBM320
	printf("%s:fbm320_init() succeeded!\n", __func__);
#endif
	return err;

err_chip_id_chk:
#ifdef DEBUG_FBM320
	printf("%s:fbm320_init() failed!; fbm320_ID:%#x,err:%d\n", __func__, fbm320_barom.chip_id, err);
#endif
	return err;
err_version_identification:
#ifdef DEBUG_FBM320
	printf("%s:fbm320_init() failed!; fbm320 version:%#x,err:%d\n", __func__, fbm320_barom.hw_ver, err);
#endif
	return err;
err_read_otp:
#ifdef DEBUG_FBM320
	printf("%s:fbm320_init() failed!; fbm320 otp reading failed!,err:%d\n", __func__, err);
#endif
	return err;
}

int32_t fbm320_read_raw_t(void)
{
	return barom->raw_temperature;
}
/**
 * @brief      API for read real temperature value in unit of degree Celsius
 *
 * @return     { temperature value in unit of degree Celsius }
 */
float fbm320_read_temperature(void)
{
	fbm320_calculation(barom);
	return barom->real_temperature * 0.01;
}

int32_t fbm320_read_raw_p(void)
{
	return barom->raw_pressure;
}
/**
 * @brief      API for read real pressure value in unit of Pa
 *
 * @return     { pressure value in unit of Pa }
 */
float fbm320_read_pressure(void)
{
	fbm320_calculation(barom);
	return barom->real_pressure;
}
/**
 * @brief      API for read real temperature and pressure values
 *             stored in fbm320_data structure
 *
 * @param      real_pressure     The pointer for saving real pressure value
 *                               Pressure unit: Pa
 * @param      real_temperature  The pointer for saving real temperature value
 *                               Temperature unit: 0.01 degree Celsius
 */
void fbm320_read_data(int32_t *real_pressure, int32_t *real_temperature)
{
	fbm320_calculation(barom);
	*real_pressure = barom->real_pressure;
	*real_temperature = barom->real_temperature;
	return;
}

/**
 * @brief      { This api ignite a measurement procedure. It writes data into
 *               the register of FBM320_TAKE_MEAS_REG. }
 *
 * @param      barom  pointer of fbm320 data structure
 *
 * @return     { return of bus_write() }
 */
static int32_t fbm320_startMeasure_temp(struct fbm320_data *barom)
{
	int32_t err;
	uint8_t bus_wr_data;

	bus_wr_data = FBM320_MEAS_TEMP;
	err = barom->bus_write(FBM320_TAKE_MEAS_REG, sizeof(uint8_t), &bus_wr_data);

	return err;
}
/**
 * @brief      { This api gets the data from the registers of FBM320_READ_MEAS_REG_U
 *               , FBM320_READ_MEAS_REG_L and FBM320_READ_MEAS_REG_XL. And the data are
 *               stored in "barom->raw_temperature". }
 *
 * @param      barom  pointer of fbm320 data structure
 *
 * @return     { return of bus_read() }
 */
static int32_t fbm320_get_raw_temperature(struct fbm320_data *barom)
{
	int32_t err;
	uint8_t buf[3] = {0};

	err = barom->bus_read(FBM320_READ_MEAS_REG_U, 3 * sizeof(uint8_t), buf);
	barom->raw_temperature = ((uint32_t)buf[0] << 16) + ((uint32_t)buf[1] << 8) + buf[2];

#ifdef DEBUG_FBM320
	printf("%s: uncompensated temperature: %d\n", DEVICE_NAME, barom->raw_temperature);
#endif
	return err;
}
/**
 * @brief      { This api ignite a measurement procedure. It writes data into
 *               the register of FBM320_TAKE_MEAS_REG. }
 *
 * @param      barom  pointer of fbm320 data structure
 *
 * @return     { return of bus_write() }
 */
static int32_t fbm320_startMeasure_press(struct fbm320_data *barom)
{
	int32_t err;
	uint8_t bus_wr_data;

	bus_wr_data = barom->cmd_start_p;
	err = barom->bus_write(FBM320_TAKE_MEAS_REG, sizeof(uint8_t), &bus_wr_data);

	return err;
}
/**
 * @brief      { This api gets the data from the registers of FBM320_READ_MEAS_REG_U
 *               , FBM320_READ_MEAS_REG_L and FBM320_READ_MEAS_REG_XL. And the data are
 *               stored in "barom->raw_temperature". }
 *
 * @param      barom  pointer of fbm320 data structure
 *
 * @return     { return of bus_read() }
 */
static int32_t fbm320_get_raw_pressure(struct fbm320_data *barom)
{
	int32_t err;
	uint8_t buf[3] = {0};

	err = barom->bus_read(FBM320_READ_MEAS_REG_U, 3 * sizeof(uint8_t), buf);
	barom->raw_pressure = ((uint32_t)buf[0] << 16) + ((uint32_t)buf[1] << 8) + buf[2];

#ifdef DEBUG_FBM320
	printf("%s: uncompensated pressure:  %d\n", DEVICE_NAME, barom->raw_pressure);
#endif

	return err;
}
/**
 * @brief      { API for reading calibration data saved in OTP memory }
 *
 * @param      barom  FBM320 data structure
 *
 * @return     { description_of_the_return_value }
 */
static int32_t fbm320_read_store_otp_data(struct fbm320_data *barom)
{
	struct fbm320_calibration_data *cali = &(barom->calibration);
	int32_t status;
	uint16_t R[10] = {0};
	uint8_t tmp[FBM320_CALIBRATION_DATA_LENGTH] = {0};

	status = barom->bus_read(FBM320_CALIBRATION_DATA_START0,
	                         (FBM320_CALIBRATION_DATA_LENGTH - 2) * sizeof(uint8_t),
	                         (uint8_t *)tmp);

	if (status < 0)
		goto exit;
	status = barom->bus_read(FBM320_CALIBRATION_DATA_START1, sizeof(uint8_t), (uint8_t *)tmp + 18 );
	if (status < 0)
		goto exit;
	status = barom->bus_read(FBM320_CALIBRATION_DATA_START2, sizeof(uint8_t), (uint8_t *)tmp + 19);
	if (status < 0)
		goto exit;
	/* Read OTP data here */
	R[0] = (tmp[0] << 8 | tmp[1]);
	R[1] = (tmp[2] << 8 | tmp[3]);
	R[2] = (tmp[4] << 8 | tmp[5]);
	R[3] = (tmp[6] << 8 | tmp[7]);
	R[4] = (tmp[8] << 8 | tmp[9]);
	R[5] = (tmp[10] << 8 | tmp[11]);
	R[6] = (tmp[12] << 8 | tmp[13]);
	R[7] = (tmp[14] << 8 | tmp[15]);
	R[8] = (tmp[16] << 8 | tmp[17]);
	R[9] = (tmp[18] << 8 | tmp[19]);

	/* Coefficient reconstruction */
	cali->C0 = R[0] >> 4;
	cali->C1 = ((R[1] & 0xFF00) >> 5 ) | (R[2] & 0x7);
	cali->C2 = ((R[1] & 0xFF) << 1 ) | (R[4] & 0x1);
	cali->C3 = R[2] >> 3;
	cali->C4 = ((uint32_t)R[3] << 2) | (R[0] & 0x3);
	cali->C5 = R[4] >> 1;
	cali->C6 = R[5] >> 3;
	cali->C7 = ((uint32_t)R[6] << 3) | (R[5] & 0x7);
	cali->C8 = R[7] >> 3;
	cali->C9 = R[8] >> 2;
	cali->C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 0x3);
	cali->C11 = R[9] & 0xFF;
	cali->C12 = ((R[0] & 0xC) << 1) | (R[7] & 0x7);

#if defined(DEBUG_FBM320) || defined(MSG_LOG)
	printf("%s: R0= %#x\n", DEVICE_NAME, R[0]);
	printf("%s: R1= %#x\n", DEVICE_NAME, R[1]);
	printf("%s: R2= %#x\n", DEVICE_NAME, R[2]);
	printf("%s: R3= %#x\n", DEVICE_NAME, R[3]);
	printf("%s: R4= %#x\n", DEVICE_NAME, R[4]);
	printf("%s: R5= %#x\n", DEVICE_NAME, R[5]);
	printf("%s: R6= %#x\n", DEVICE_NAME, R[6]);
	printf("%s: R7= %#x\n", DEVICE_NAME, R[7]);
	printf("%s: R8= %#x\n", DEVICE_NAME, R[8]);
	printf("%s: R9= %#x\n", DEVICE_NAME, R[9]);
	printf("%s: C0= %d\n", DEVICE_NAME, cali->C0);
	printf("%s: C1= %d\n", DEVICE_NAME, cali->C1);
	printf("%s: C2= %d\n", DEVICE_NAME, cali->C2);
	printf("%s: C3= %d\n", DEVICE_NAME, cali->C3);
	printf("%s: C4= %d\n", DEVICE_NAME, cali->C4);
	printf("%s: C5= %d\n", DEVICE_NAME, cali->C5);
	printf("%s: C6= %d\n", DEVICE_NAME, cali->C6);
	printf("%s: C7= %d\n", DEVICE_NAME, cali->C7);
	printf("%s: C8= %d\n", DEVICE_NAME, cali->C8);
	printf("%s: C9= %d\n", DEVICE_NAME, cali->C9);
	printf("%s: C10= %d\n", DEVICE_NAME, cali->C10);
	printf("%s: C11= %d\n", DEVICE_NAME, cali->C11);
	printf("%s: C12= %d\n", DEVICE_NAME, cali->C12);
#endif

exit:
	return status;
}
/**
 * @brief      { API for reading hardware version }
 *
 * @param      barom  FBM320 data structure
 *
 * @return     { description_of_the_return_value }
 */
static int32_t fbm320_version_identification(struct fbm320_data *barom)
{
	int32_t err;
	uint8_t buf[2] = {0};
	uint8_t version = 0;
	uint8_t bus_wr_data;

	bus_wr_data = FBM320_SOFTRESET_CMD;
	barom->bus_write(FBM320_SOFTRESET_REG, sizeof(uint8_t), &bus_wr_data);
	barom->delay_usec(1000 * 15); /* The minimum start up time of fbm320 is
15ms */
	err = barom->bus_read(FBM320_TAKE_MEAS_REG, sizeof(uint8_t), buf);
	err = barom->bus_read(FBM320_VERSION_REG, sizeof(uint8_t), buf + 1);

	version = ((buf[0] & 0xC0) >> 6) | ((buf[1] & 0x70) >> 2);
#if defined(DEBUG_FBM320) || defined(MSG_LOG)
	printf("%s: The value of version: %#x\n", __func__, version);
#endif

	switch (version)	{
	case hw_ver_b1:
		barom->hw_ver = hw_ver_b1;
#if defined(DEBUG_FBM320) || defined(MSG_LOG)
		printf("%s: The version of sensor is B1.\n", __func__);
#endif
		break;
	default:
		barom->hw_ver = hw_ver_unknown;
#if defined(DEBUG_FBM320) || defined(MSG_LOG)
		printf("%s: The version of sensor is unknown.\n", __func__);
#endif
		break;
	}
	return err;
}
static int32_t fbm320_set_oversampling_rate(struct fbm320_data *barom
        , enum fbm320_osr osr_setting)
{
	uint8_t reg_addr;
	uint8_t data_buf;

	barom->oversampling_rate = osr_setting;
#ifdef DEBUG_FBM320
	printf("%s:Setting of oversampling_rate:%#x\r\n", __func__, barom->oversampling_rate);
#endif

	/* Setting conversion time for pressure measurement */
	switch (osr_setting) {
	case osr_1024:
		barom->cnvTime_press = FBM320_CONVERSION_usTIME_OSR1024;
		barom->cmd_start_p = FBM320_MEAS_PRESS_OVERSAMP_0;
		break;
	case osr_2048:
		barom->cnvTime_press = FBM320_CONVERSION_usTIME_OSR2048;
		barom->cmd_start_p = FBM320_MEAS_PRESS_OVERSAMP_1;
		break;
	case osr_4096:
		barom->cnvTime_press = FBM320_CONVERSION_usTIME_OSR4096;
		barom->cmd_start_p = FBM320_MEAS_PRESS_OVERSAMP_2;
		break;
	case osr_8192:
		barom->cnvTime_press = FBM320_CONVERSION_usTIME_OSR8192;
		barom->cmd_start_p = FBM320_MEAS_PRESS_OVERSAMP_3;
		break;
	case osr_16384:
		barom->cnvTime_press = FBM320_CONVERSION_usTIME_OSR16384;
		reg_addr = 0xa6;
		barom->bus_read(reg_addr, sizeof(uint8_t), &data_buf);
		data_buf &= 0xf8;
		data_buf |= 0x6;
		barom->bus_write(reg_addr, sizeof(uint8_t), &data_buf);
		barom->cmd_start_p = FBM320_MEAS_PRESS_OVERSAMP_2;
		barom->bus_read(0xA6, sizeof(uint8_t), &data_buf);
#ifdef DEBUG_FBM320
		printf("reg_0xA6:%#x\n\r", data_buf);
#endif
		break;
	}
	/* Setting covversion time for temperature measurement */
	barom->cnvTime_temp = FBM320_CONVERSION_usTIME_OSR1024;

	return 0;
}
static int32_t fbm320_chipid_check(struct fbm320_data *barom)
{
	int32_t err;
	uint8_t chip_id_read;

	err = barom->bus_read(FBM320_CHIP_ID_REG, sizeof(uint8_t), &chip_id_read);
#ifdef DEBUG_FBM320
	printf("%s: chip_id reading is %#x \n", __func__, chip_id_read);
#endif

	if (chip_id_read != FBM320_CHIP_ID) {
		err = -1;
		return err;
	} else {
		barom->chip_id = chip_id_read;
		return err = 0;
	}
}
/**
 * @brief      { API for triggering measurement procedure and updating
 *               the temperature and pressure data in fbm320_data structure. }
 */
void fbm320_update_data(void)
{
	static uint32_t t_start_flag = 0;
	static uint32_t p_start_flag = 0;
	static uint32_t tick_current;
	static uint32_t tick_last;
	static uint32_t tick_diff;

	tick_current = TMR0_Ticks;
	tick_diff = tick_current - tick_last;

	if (t_start_flag == 0 && !fbm320_update_rdy) {
#ifdef DEBUG_FBM320
		printf("start t_measurement\r\n");
#endif
		fbm320_startMeasure_temp(barom);
		t_start_flag = 1;
		tick_last = TMR0_Ticks;
	} else if ((tick_diff * 1000 > barom->cnvTime_temp ) && (p_start_flag == 0)) {
#ifdef DEBUG_FBM320
		printf("start p_measurement\r\n");
#endif
		fbm320_get_raw_temperature(barom);
		fbm320_startMeasure_press(barom);
		p_start_flag = 1;
		tick_last = TMR0_Ticks;
	} else if (tick_diff * 1000 > barom->cnvTime_press ) {
#ifdef DEBUG_FBM320
		printf("read pressure\r\n");
#endif
		fbm320_get_raw_pressure(barom);
		t_start_flag = 0;
		p_start_flag = 0;
		tick_current = 0;
		tick_last = 0;
		TMR0_Ticks = 0;
		fbm320_update_rdy = 1;
	}
#ifdef DEBUG_FBM320
	printf("tick_current:%d\r\n", tick_current);
	printf("tick_last:%d\r\n", tick_last);
	printf("FBM320 is updating %d\r\n", TMR0_Ticks);
#endif
	return ;
}
/**
 * @brief      { API for calculating real temperature and pressure values.
 *               The results are stored in fbm320_data structure.
 *               "barom->real_temperature" is represented real temperature value.
 *               "barom->real_temperature" is in uint of 0.01 drgree Celsius.
 *               "barom->real_pressure" is represented real pressure value.
 *               "barom->real_pressure" is in unit of Pa. }
 *
 * @param      barom  pointer of fbm320 data structure
 *
 * @return     { description_of_the_return_value }
 */
static int fbm320_calculation(struct fbm320_data *barom)
{
	struct fbm320_calibration_data *cali = &barom->calibration;
	int32_t X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26;
	int32_t PP1, PP2, PP3, PP4, CF, X31, X32;
	int32_t RT, RP, UT, UP, DT, DT2;

	/* calculation for real temperature value*/
	UT = barom->raw_temperature;
	DT = ((UT - 8388608) >> 4) + (cali->C0 << 4);
	X01 = (cali->C1 + 4498L) * DT >> 1;
	X02 = ((((cali->C2 - 256L) * DT) >> 14) * DT) >> 4;
	X03 = (((((cali->C3 * DT) >> 18) * DT) >> 18) * DT);
	DT2 = (X01 + X02 + X03) >> 12;
	RT =  ((2500L << 15) - X01 - X02 - X03) >> 15;
	/* calculation for real pressure value*/
	UP = barom->raw_pressure;
	X11 = ((cali->C5 - 15446L) * DT2);
	X12 = ((((cali->C6 - 4096L) * DT2) >> 16) * DT2) >> 4;
	X13 = ((X11 + X12) >> 11) + ((cali->C4 - 122684) << 4);
	X21 = ((cali->C8 + 1528L) * DT2) >> 11;
	X22 = (((cali->C9 * DT2) >> 17) * DT2) >> 13;
	X23 = (X22 >= X21) ? (X22 - X21) : (X21 - X22);

	X24 = (X23 >> 11) * (cali->C7 + 596352);
	X25 = ((X23 & 0x7FF) * (cali->C7 + 596352)) >> 11;
	X26 = (X21 >= X22 ? ((0 - X24 - X25) >> 9) : ((X24 + X25) >> 9)) + cali->C7 + 596352;

	PP1 = (((UP - 8388608) >> 1) - X13) >> 4;
	PP2 = ((X26 >> 12) * PP1) >> 1;
	PP3 = ((X26 & 0xFFF) * PP1) >> 13;
	PP4 = (PP2 + PP3) >> 3;
	CF = (2097152 + cali->C12 * DT2) >> 2;
	X31 = (((CF * cali->C10) >> 22) * PP4) >> 6;
	X32 = (((((CF * cali->C11) >> 20) * PP4) >> 22) * PP4);
	RP = ((X31 + X32) >> 11) + PP4 + 100000;

	barom->real_temperature = RT; //uint:0.01 degree Celsius
	barom->real_pressure = RP; //uint: Pa

#ifdef DEBUG_FBM320
	printf("%s: calibrated pressure: %d\n", DEVICE_NAME, RP);
#endif

	return 0;
}
