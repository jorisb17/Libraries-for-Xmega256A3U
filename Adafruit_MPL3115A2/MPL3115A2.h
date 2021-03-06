/*!
 * @file MPL3115A2.h
 *
 * This version of the adafruit library for the MPL3115A2 breakout has been
 * rewritten for the xmegas256A3U by:
 * Joris Bruinsma
 * It is designed specifically to work with the Adafruit MPL3115A2 breakout:
 * https://www.adafruit.com/products/1893
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include <avr/io.h>
#include "i2c.h"

#define MPL3115A2_ADDRESS (0x60) ///< default I2C address 1100000
#define MPL3115A2_REGISTER_STARTCONVERSION (0x12) ///< start conversion

/** MPL3115A2 registers **/
enum {
  MPL3115A2_REGISTER_STATUS = (0x00),

  MPL3115A2_REGISTER_PRESSURE_MSB = (0x01),
  MPL3115A2_REGISTER_PRESSURE_CSB = (0x02),
  MPL3115A2_REGISTER_PRESSURE_LSB = (0x03),

  MPL3115A2_REGISTER_TEMP_MSB = (0x04),
  MPL3115A2_REGISTER_TEMP_LSB = (0x05),

  MPL3115A2_REGISTER_DR_STATUS = (0x06),

  MPL3115A2_OUT_P_DELTA_MSB = (0x07),
  MPL3115A2_OUT_P_DELTA_CSB = (0x08),
  MPL3115A2_OUT_P_DELTA_LSB = (0x09),

  MPL3115A2_OUT_T_DELTA_MSB = (0x0A),
  MPL3115A2_OUT_T_DELTA_LSB = (0x0B),

  MPL3115A2_WHOAMI = (0x0C),

  MPL3115A2_BAR_IN_MSB = (0x14),
  MPL3115A2_BAR_IN_LSB = (0x15),
};

/** MPL3115A2 status register bits **/
enum {
  MPL3115A2_REGISTER_STATUS_TDR = 0x02,
  MPL3115A2_REGISTER_STATUS_PDR = 0x04,
  MPL3115A2_REGISTER_STATUS_PTDR = 0x08,
};

/** MPL3115A2 PT DATA register bits **/
enum {
  MPL3115A2_PT_DATA_CFG = 0x13,
  MPL3115A2_PT_DATA_CFG_TDEFE = 0x01,
  MPL3115A2_PT_DATA_CFG_PDEFE = 0x02,
  MPL3115A2_PT_DATA_CFG_DREM = 0x04,
};

/** MPL3115A2 control registers **/
enum {

  MPL3115A2_CTRL_REG1 = (0x26),
  MPL3115A2_CTRL_REG2 = (0x27),
  MPL3115A2_CTRL_REG3 = (0x28),
  MPL3115A2_CTRL_REG4 = (0x29),
  MPL3115A2_CTRL_REG5 = (0x2A),
};

/** MPL3115A2 control register bits **/
enum {
  MPL3115A2_CTRL_REG1_SBYB = 0x01,
  MPL3115A2_CTRL_REG1_OST = 0x02,
  MPL3115A2_CTRL_REG1_RST = 0x04,
  MPL3115A2_CTRL_REG1_RAW = 0x40,
  MPL3115A2_CTRL_REG1_ALT = 0x80,
  MPL3115A2_CTRL_REG1_BAR = 0x00,
};

/** MPL3115A2 oversample values **/
enum {
  MPL3115A2_CTRL_REG1_OS1 = 0x00,
  MPL3115A2_CTRL_REG1_OS2 = 0x08,
  MPL3115A2_CTRL_REG1_OS4 = 0x10,
  MPL3115A2_CTRL_REG1_OS8 = 0x18,
  MPL3115A2_CTRL_REG1_OS16 = 0x20,
  MPL3115A2_CTRL_REG1_OS32 = 0x28,
  MPL3115A2_CTRL_REG1_OS64 = 0x30,
  MPL3115A2_CTRL_REG1_OS128 = 0x38,
};

/*!
 *  @brief  Struct that stores state for interacting with MPL3115A2
 * altimeter
 */
typedef struct MPL3115A2_t{
	uint8_t mode;
	TWI_t *twi;
	union{
		struct{
			uint8_t SBYB : 1;
			uint8_t OST : 1;
			uint8_t RSTT : 1;
			uint8_t OS : 3;
			uint8_t RAW : 1;
			uint8_t ALT : 1;
		}bit;
		uint8_t reg;
	}_ctrl_reg1;
}MPL3115A2;

bool MPL3115A2_begin(TWI_t *twi, MPL3115A2 *s);
float MPL3115A2_getPressure(MPL3115A2 *s);
float MPL3115A2_getAltitude(MPL3115A2 *s);
void MPL3115A2_setSeaPressure(MPL3115A2 *s, float pascal);
float MPL3115A2_getTemperature(MPL3115A2 *s);