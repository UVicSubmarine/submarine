
#include <avr/io.h>
#include <util/delay.h>
#include "MS5837.h"
#include "i2c.h"
#include "USART.h"

#define MS5837_ADDR               0xEC
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

//this function sets up the MS5837 and reads and checks it's calibration data
//it takes an array of 7 16-bit words to return the calibration data in.
uint8_t initMS5837(uint16_t* calib_data){

  //initialize i2c (might move this to main())
  i2c_init();

  //reset device
  i2c_transmit(MS5837_ADDR, (uint8_t*)MS5837_RESET, 1);
  _delay_ms(10);

  //read calibration data drom device
  printString("Calibration Data: ");
  for(uint8_t i = 0; i < 7; i++){
    i2c_start(MS5837_ADDR);
    i2c_write(MS5837_PROM_READ + i*2);
    i2c_stop();

    i2c_start(MS5837_ADDR | I2C_READ);
    calib_data[i] = (i2c_read_ack() << 8 ) | i2c_read_nack();
    i2c_stop();
    printHexByte(calib_data[i] >> 8);
    printHexByte(calib_data[i]);
  }

  //check CRC on calibration data
  printString("\nCRC Check: ");
  uint8_t crcRead = calib_data[0] >> 12;
  uint8_t crcCalculated = crc4(calib_data);

  if(crcRead == crcCalculated){
    printString(":)\n");
    return 1;
  }

  printString(" :(\n");
  return 0;
}


//CRC check on MS5837 data
uint8_t crc4(uint16_t n_prom[]){
  uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}
