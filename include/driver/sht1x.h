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


#ifndef SHT1X_H_
#define SHT1X_H_


#define I2C_SDA_MUX PERIPHS_IO_MUX_GPIO4_U
#define I2C_SDA_FUNC FUNC_GPIO5
#define I2C_SDA_PIN 5

#define I2C_SCK_MUX PERIPHS_IO_MUX_GPIO5_U
#define I2C_SCK_FUNC FUNC_GPIO4
#define I2C_SCK_PIN 4

typedef enum sht1x_ack {
	withACK,
	withoutACK
} sht1x_ackT ;

typedef enum sht1x_cmd {
	MEASURE_TEMP = 0x03,
	MEASURE_HUMI = 0x05,
	STATUS_REG_W = 0x06,
	STATUS_REG_R = 0x07,
	SW_RESET	 = 0x1e
} sht1x_cmdT ;

void sht1x_init();
uint8_t sht11_measure(const sht1x_cmdT mode, uint16_t *result);
void calc_sth11(float *p_humidity ,float *p_temperature);

#endif /* SHT1X_H_ */
