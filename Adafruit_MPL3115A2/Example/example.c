#define F_CPU 2000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>

#include "serialF0.h"                             // or use serialF0.h

#define BAUD_100K        100000UL
#include "i2c.h"
#include "MPL3115A2.h"

MPL3115A2 sensor;

int main(void)
{
	i2c_init(&TWIE, TWI_BAUD(F_CPU, BAUD_100K));
	PORTE.DIRSET    = PIN1_bm|PIN0_bm;            // SDA 0 SCL 1
	PORTE.PIN0CTRL  = PORT_OPC_WIREDANDPULL_gc;   // Pullup SDA
	PORTE.PIN1CTRL  = PORT_OPC_WIREDANDPULL_gc;   // Pullup SCL

	init_stream(F_CPU);
	sei();
	
	while(1) {
		if(!(MPL3115A2_begin(&TWIE, &sensor))){
			printf("Couldn't find sensor\n");
		}
		  float pascals = MPL3115A2_getPressure(&sensor);
		  // Our weather page presents pressure in Inches (Hg)
		  // Use http://www.onlineconversion.com/pressure.htm for other units
		  printf("%f", pascals/3377);
		  printf(" Inches (Hg)\n");

		 float altm = MPL3115A2_getAltitude(&sensor);
		 printf("%f", altm); printf(" meters\n");

		 float tempC = MPL3115A2_getTemperature(&sensor);
		 printf("%f", tempC); printf("*C\n");

		_delay_ms(250);
	}
}
