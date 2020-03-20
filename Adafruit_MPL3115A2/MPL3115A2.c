/*!
 *	This version of the adafruit library for the MPL3115A2 breakout has been rewritten for
 *  the xmegas256A3U by:
 *	Joris Bruinsma
 *
 *  @file MPL3115A2.c
 *
 *  @mainpage Adafruit MPL3115A2 alitmeter
 *
 *  @section intro_sec Introduction
 *
 *  This is the documentation for Adafruit's MPL3115A2 driver for the
 *  Arduino platform.  It is designed specifically to work with the
 *  Adafruit MPL3115A2 breakout: https://www.adafruit.com/products/1893
 *
 *  These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 *  to interface with the breakout.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 *  @section dependencies Dependencies
 *
 *  @section author Author
 *
 *  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 *  @section license License
 *
 *  BSD license, all text here must be included in any redistribution.
 *
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "i2c.h"
#include "MPL3115A2.h"

static void write8(TWI_t *twi, uint8_t a, uint8_t d);
static uint8_t read8(TWI_t *twi, uint8_t a);

bool MPL3115A2_begin(TWI_t *twi, MPL3115A2 *s){
	s->twi = twi;
	uint8_t whoami = read8(twi, MPL3115A2_WHOAMI);
	i2c_stop(s->twi);
	if(whoami != 0xC4){
		return false;
	}
	write8(s->twi, MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);
	_delay_ms(10);
	
	while(read8(s->twi, MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_RST)
	_delay_ms(10);

	s->_ctrl_reg1.reg = MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT;
	
	write8(s->twi, MPL3115A2_CTRL_REG1, s->_ctrl_reg1.reg);

	write8(s->twi, MPL3115A2_PT_DATA_CFG, MPL3115A2_PT_DATA_CFG_TDEFE |
	MPL3115A2_PT_DATA_CFG_PDEFE |
	MPL3115A2_PT_DATA_CFG_DREM);
	
	return true;
}

float MPL3115A2_getPressure(MPL3115A2 *s) {
	uint32_t pressure;
	while (read8(s->twi, MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_OST)
	_delay_ms(10);
	
	s->_ctrl_reg1.bit.ALT = 0;
	write8(s->twi, MPL3115A2_CTRL_REG1, s->_ctrl_reg1.reg);
	
	s->_ctrl_reg1.bit.OST = 1;
	write8(s->twi, MPL3115A2_CTRL_REG1, s->_ctrl_reg1.reg);

	uint8_t sta = 0;
	while (!(sta & MPL3115A2_REGISTER_STATUS_PDR)) {
		sta = read8(s->twi, MPL3115A2_REGISTER_STATUS);
		_delay_ms(10);
	}
	i2c_start(s->twi, MPL3115A2_ADDRESS, I2C_WRITE);
	i2c_write(s->twi, MPL3115A2_REGISTER_PRESSURE_MSB);
	i2c_restart(s->twi, MPL3115A2_ADDRESS, I2C_READ);
	pressure = i2c_read(s->twi, I2C_ACK);       // receive DATA
	pressure <<= 8;
	pressure |= i2c_read(s->twi, I2C_ACK); // receive DATA
	pressure <<= 8;
	pressure |= i2c_read(s->twi, I2C_NACK); // receive DATA
	pressure >>= 4;
	i2c_stop(s->twi);
	float baro = pressure;
	baro /= 4.0;
	return baro;
}

/*!
 *  @brief  Gets the floating-point altitude value
 *  @return altitude reading as a floating-point value
 */
float MPL3115A2_getAltitude(MPL3115A2 *s) {
	int32_t alt;
	
	while (read8(s->twi, MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_OST)
    _delay_ms(10);

	s->_ctrl_reg1.bit.ALT = 1;
	write8(s->twi, MPL3115A2_CTRL_REG1, s->_ctrl_reg1.reg);
	
	s->_ctrl_reg1.bit.OST = 1;
	write8(s->twi, MPL3115A2_CTRL_REG1, s->_ctrl_reg1.reg);

	uint8_t sta = 0;
	while (!(sta & MPL3115A2_REGISTER_STATUS_PDR)) {
		sta = read8(s->twi, MPL3115A2_REGISTER_STATUS);
		_delay_ms(10);
	}
	i2c_start(s->twi, MPL3115A2_ADDRESS, I2C_WRITE);
	i2c_write(s->twi, MPL3115A2_REGISTER_PRESSURE_MSB);
	i2c_restart(s->twi, MPL3115A2_ADDRESS, I2C_READ);

	alt = ((uint32_t)i2c_read(s->twi, I2C_ACK)) << 24;  // receive DATA
	alt |= ((uint32_t)i2c_read(s->twi, I2C_ACK))  << 16; // receive DATA
	alt |= ((uint32_t)i2c_read(s->twi, I2C_NACK))  << 8;  // receive DATA
	i2c_stop(s->twi);
	float altitude = alt;
	altitude /= 65536.0;
	return altitude;
}

/*!
 *  @brief  Set the local sea level barometric pressure
 *  @param pascal the pressure to use as the baseline
 */
void MPL3115A2_setSeaPressure(MPL3115A2 *s, float pascal) {
	uint16_t bar = pascal / 2;
	i2c_start(s->twi, MPL3115A2_ADDRESS, I2C_WRITE);
	i2c_write(s->twi, MPL3115A2_REGISTER_PRESSURE_MSB);
	i2c_write(s->twi, (uint8_t)(bar >> 8));
	i2c_write(s->twi, (uint8_t)bar);
	i2c_stop(s->twi);
}

/*!
 *  @brief  Gets the floating-point temperature in Centigrade
 *  @return temperature reading in Centigrade as a floating-point value
 */
float MPL3115A2_getTemperature(MPL3115A2 *s) {
	 int16_t t;	
  	
	 s->_ctrl_reg1.bit.OST = 1;
	write8(s->twi, MPL3115A2_CTRL_REG1, s->_ctrl_reg1.reg);

	uint8_t sta = 0;
	while (!(sta & MPL3115A2_REGISTER_STATUS_TDR)) {
		sta = read8(s->twi, MPL3115A2_REGISTER_STATUS);
		_delay_ms(10);
	}
	i2c_start(s->twi, MPL3115A2_ADDRESS, I2C_WRITE);
	i2c_write(s->twi, MPL3115A2_REGISTER_TEMP_MSB);
	i2c_restart(s->twi, MPL3115A2_ADDRESS, I2C_READ);
	t = i2c_read(s->twi, I2C_ACK);              // receive DATA
	t <<= 8;
	t |= i2c_read(s->twi, I2C_NACK); // receive DATA
	t >>= 4;
	i2c_stop(s->twi);	

	if (t & 0x800) {
		t |= 0xF000;
	}
	
	float temp = t;
	temp /= 16.0;
	return temp;
}

/*!
 *  @brief  read 1 byte of data at the specified address
 *  @param  a 
 *          the address to read
 *  @return the read data byte
 */
static uint8_t read8(TWI_t *twi, uint8_t a) {
	uint8_t res = 0;
	i2c_start(twi, MPL3115A2_ADDRESS, I2C_WRITE);
	i2c_write(twi, a);
	i2c_restart(twi, MPL3115A2_ADDRESS, I2C_READ);
	res = i2c_read(twi, I2C_NACK);
	i2c_stop(twi);
	return res;
}

/*!
 *  @brief  write a byte of data to the specified address
 *  @param  a 
 *          the address to write to
 *  @param  d 
 *          the byte to write
 */
static void write8(TWI_t *twi, uint8_t a, uint8_t d) {
	i2c_start(twi, MPL3115A2_ADDRESS, I2C_WRITE);
	i2c_write(twi, a);
	i2c_write(twi, d);
	i2c_stop(twi);
}