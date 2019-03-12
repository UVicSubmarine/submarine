
#include <avr/io.h>
#include <util/delay.h>
#include "MS5837.h"
#include "i2c.h"
#include "USART.h"

#define MS5837_ADDR               0b11101100
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

void initMS5837(void){

  DDRB |= (1<<PB0);
  PORTB |= (1 << PB0);


  i2c_init();

  //reset the device
  i2c_start(0b11101100);
  i2c_write(MS5837_RESET);
  i2c_stop();

  _delay_ms(10);

  //send PROM request
  i2c_start(MS5837_ADDR);
  i2c_write(MS5837_PROM_READ);
  i2c_stop();

  _delay_ms(10);

  //start reading from device
  i2c_start(MS5837_ADDR | I2C_READ);
  i2c_read_ack();
  i2c_stop();

  /*
  uint8_t* data = 0;

    if(!i2c_receive(MS5837_ADDR, data, 2)){
      printHexByte(*data);
    } else {
      printString("Bad data");
    }
  }
  */
  printString("Data collected \n");
}
