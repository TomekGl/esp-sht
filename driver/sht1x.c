/**
 ******************************************************************************
 * @addtogroup SHT11 driver
 * @brief Bit-banging driver for Sensirion SHT1x sensors
 *
 * @{
 * @file       main.c
 * @author     Tomasz Głuch <contact@tomaszgluch.pl>
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"

#include "driver/sht1x.h"



#define SCL_HI()	(gpio_output_set(1 << I2C_SCK_PIN, 0, 1 << I2C_SCK_PIN, 0));
#define SCL_LOW()	(gpio_output_set(0, 1 << I2C_SCK_PIN, 1 << I2C_SCK_PIN, 0));
#define SDA_HI()	(gpio_output_set(1 << I2C_SDA_PIN, 0, 1 << I2C_SDA_PIN, 0));
#define SDA_LOW()	(gpio_output_set(0, 1 << I2C_SDA_PIN, 1 << I2C_SDA_PIN, 0));


/* 400kHz @ 8MHz */
//#define SHT_NOP() __NOP();__NOP();__NOP();__NOP();__NOP();
#define SHT_NOP() 	os_delay_us(4)

inline uint16_t SDA_GET() {
	return GPIO_INPUT_GET(GPIO_ID_PIN(I2C_SDA_PIN));
}

void _transmission_start(void) {
	SCL_HI();
	SHT_NOP();
	SDA_LOW();
	SHT_NOP();
	SCL_LOW();
	SHT_NOP();SHT_NOP();SHT_NOP();
	SCL_HI();
	SHT_NOP();
	SDA_HI();
	SHT_NOP();
	SCL_LOW();
}



void _reset() {
	/* 9 sck cyckles + transmission_start */
	int i;
	SDA_HI();
	SCL_LOW();
	for(i=9;i>0;i--) {
		SCL_HI();
		SHT_NOP();
		SCL_LOW();
		SHT_NOP();
	}
	_transmission_start();
}

void sht1x_init()  {

    //Disable interrupts
    ETS_GPIO_INTR_DISABLE();

    //Set pin functions
    PIN_FUNC_SELECT(I2C_SDA_MUX, I2C_SDA_FUNC);
    PIN_FUNC_SELECT(I2C_SCK_MUX, I2C_SCK_FUNC);

    //Set SDA as open drain
    GPIO_REG_WRITE(
        GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_SDA_PIN)),
        GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_SDA_PIN))) |
        GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)
    );

    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_SDA_PIN));

    //Set SCK as open drain
    GPIO_REG_WRITE(
        GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_SCK_PIN)),
        GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_SCK_PIN))) |
        GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)
    );

    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_SCK_PIN));

    //Turn interrupt back on
    ETS_GPIO_INTR_ENABLE();

	SDA_HI();
	SCL_LOW();
	_reset();

	/* 11ms */
	os_delay_us(11000);

}


uint8_t _read_byte(sht1x_ackT ack) {
	uint8_t i, val=0;
	SDA_HI();
	for (i=0x80; i>0; i/=2) {
		SCL_HI();
		SHT_NOP();
		if(SDA_GET())
			val = (val | i);
		SCL_LOW();
		SHT_NOP();
	}
	if(ack == withACK)
		SDA_LOW();
	SHT_NOP();
	SCL_HI();
	SHT_NOP();SHT_NOP();SHT_NOP();
	SCL_LOW();
	SDA_HI();
	SHT_NOP();
	return val;
}


uint8_t _write_byte(uint8_t value) {
	uint8_t i, error=0;
	SDA_HI();
	SHT_NOP();
	for (i=0x80; i>0; i>>=1) {
		if ( i & value) {
			SDA_HI();
		}
		else {
			SDA_LOW();
		}
		SCL_HI();
		SHT_NOP();
		SCL_LOW();
		SHT_NOP();
	}
	SDA_HI();
	SHT_NOP();
	SCL_HI();
	SHT_NOP();
	if (SDA_GET())
		error = 1;
	SCL_LOW();
	SHT_NOP();
	return error;
}



uint8_t _read_status(uint8_t *status) {
	uint8_t error = 0, x=0,ck=0;

	_transmission_start();
	error = _write_byte(STATUS_REG_R);
	*status = _read_byte(withACK);
	ck = _read_byte(withoutACK);
	return error;
}


/* From Sensirion appnote */
//----------------------------------------------------------------------------------
void calc_sth11(float *p_humidity, float *p_temperature)
//----------------------------------------------------------------------------------
// calculates temperature [°C] and humidity [%RH]
// input : humi [Ticks] (12 bit)
// temp [Ticks] (14 bit)
// output: humi [%RH]
// temp [°C]
{
	const float C1 = -2.0468;
	// for 12 Bit RH
	const float C2 = +0.0367;
	// for 12 Bit RH
	const float C3 = -0.0000015955;
	// for 12 Bit RH
	const float T1 = +0.01;
	// for 12 Bit RH
	const float T2 = +0.00008;
	// for 12 Bit RH

	float rh = *p_humidity;
	float t = *p_temperature;
	float rh_lin;
	float rh_true;
	float t_C;

	t_C = t * 0.01 - 39.65;
	//calc. temperature[°C]from 14 bit temp.ticks @5V
	rh_lin = C3 * rh * rh + C2 * rh + C1;
	//calc. humidity from ticks to [%RH]
	rh_true = (t_C - 25) * (T1 + T2 * rh) + rh_lin;
	//calc. temperature compensated humidity [%RH]
	if (rh_true > 100)
		rh_true = 100;
	//cut if the value is outside of
	if (rh_true < 0.1)
		rh_true = 0.1;
	//the physical possible range
	*p_temperature = t_C;
	*p_humidity = rh_true;
	//return temperature [°C]
	//return humidity[%RH]
}

/*
//--------------------------------------------------------------------
float calc_dewpoint(float h,float t)
//--------------------------------------------------------------------
// calculates dew point
// input: humidity [%RH], temperature [°C]
// output: dew point [°C]
{ float k,dew_point ;
	k = (log10(h)-2)/0.4343 + (17.62*t)/(243.12+t);
	dew_point = 243.12*k/(17.62-k);
	return dew_point;
}
 */

uint8_t sht11_crc(uint8_t crc, uint8_t byte) {
	crc ^= byte;
	uint8_t i;
	for(i=0; i<8; i++)
	   crc = (crc << 1) ^ ((crc & 0x80)?0x31:0);
	return crc;

}

uint8_t sht11_reverse(uint8_t byte) {
	uint8_t r = byte;
	int s = 7;
	for (byte >>= 1; byte; byte >>= 1) {
		r <<= 1;
		r |= byte & 1;
		s--;
	}
	r <<= s; /* Shift when v's highest bits are zero */
	return r;
}

uint8_t sht11_measure(const sht1x_cmdT mode, uint16_t *result) {
	uint8_t error = 0;
	int i;
	uint8_t crc;
	_read_status(&crc); //Initial CRC value is status register
	_transmission_start();
	error = _write_byte(mode);
	//TODO Timeout
	for (i=655035;i>0;i--) {
		if (SDA_GET() == 0)
			break;
	}
	if (SDA_GET())
		error += 1;

	uint8_t a,b;
	a = _read_byte(withACK);
	b = _read_byte(withACK);
	crc = sht11_crc(crc, mode);
	crc = sht11_crc(crc, a);
	crc = sht11_crc(crc, b);
	crc = sht11_reverse(crc);
	if (_read_byte(withoutACK) != crc) {
		error += 1;
		_reset();
	}
	*result = a<<8|b;

	return error;
}



