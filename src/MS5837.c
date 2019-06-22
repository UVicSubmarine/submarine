
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

#define ATMOSPHERIC_PRESSURE 101260 //Pa (kg/ms^2)
#define FLUID_DENSITY 997 //kg/cm^3
#define G 9.81 //m/s^2

//this function sets up the MS5837 and reads and checks it's calibration data
//it takes an array of 7 16-bit words to return the calibration data in.
uint8_t initMS5837(uint16_t calib_data[]){

  //initialize i2c (might move this to main())
  i2c_init();

  //reset device
  i2c_transmit(MS5837_ADDR, (uint8_t*)MS5837_RESET, 1);
  _delay_ms(10);

  //read calibration data drom device
  printString("Calibration Data: \n");
  for(uint8_t i = 0; i < 7; i++){
    i2c_start(MS5837_ADDR);
    i2c_write(MS5837_PROM_READ + i*2);
    i2c_stop();

    i2c_start(MS5837_ADDR | I2C_READ);
    calib_data[i] = (i2c_read_ack() << 8 ) | i2c_read_nack();
    i2c_stop();
    char str[20];
    sprintf(str, "[C%u] %X\n", i, calib_data[i]);
    printString(str);
  }

  //check CRC on calibration data
  printString("CRC: ");

  uint8_t crcRead = calib_data[0] >> 12;
  uint8_t crcCalculated = crc4(calib_data);


  if(crcRead == crcCalculated){
    printString("Good\n");
    return 1;
  }

  printString(" Bad\n");
  return 0;
}

//MS5837 ADC conversion and read
uint32_t conversion(uint8_t conversion_code){

  i2c_start(MS5837_ADDR);
  i2c_write(conversion_code);
  i2c_stop();

  //conversion time
  _delay_ms(20);

  //request ADC read
  i2c_start(MS5837_ADDR);
  i2c_write(MS5837_ADC_READ);
  i2c_stop();

  uint32_t output = 0;

  //read adc
  i2c_start(MS5837_ADDR | I2C_READ);
  output = i2c_read_ack();
  output = (output << 8) | i2c_read_ack();
  output = (output << 8) | i2c_read_nack();
  i2c_stop();

  return output;
}

//reads data from MS5837 and returns depth
uint32_t read_MS5837_depth(uint16_t C[]){

  uint32_t D1 = conversion(MS5837_CONVERT_D1_8192);
  uint32_t D2 = conversion(MS5837_CONVERT_D2_8192);

  uint32_t dT = 0;
  int32_t T = 0;
  int32_t P = 0;
  int64_t SENS = 0;
  int64_t OFF = 0;
  //int32_t SENSi = 0;
  //int32_t OFFi = 0;
  //int32_t Ti = 0;
  //int64_t OFF2 = 0;
  //int64_t SENS2 = 0;


  dT = D2 - ((uint32_t)C[5])*256L;
  T = 2000L+(int64_t)dT*C[6]/8388608LL;

  SENS = (int64_t)C[1]*32768l+((int64_t)C[3]*dT)/256l;
	OFF = (int64_t)C[2]*65536l+((int64_t)C[4]*dT)/128l;
	P = (D1*SENS/(2097152l)-OFF)/(8192l)*10;

  return (P - ATMOSPHERIC_PRESSURE)*100/(G*FLUID_DENSITY); // in cm
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
