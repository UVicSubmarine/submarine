
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "USART.h"
#include "i2c.h"
#include "MS5837.h"

//Submarine Controller


/* LOGISTICAL PREAMBLE */

#define VERSION "v0.1"
#define DEBUG 1 //set debug mode, enabling serial communication
//#define F_CPU 8000000UL

//breakpoint function for debugging
void breakpoint(void){
  if(DEBUG){
    cli();
    PORTB |= (1 << PB0);
    printString("\n ----- \n");
    receiveByte();
    PORTB &= ~(1 << PB0);
    sei();
  }
}

/* DECALARATIONS */
#define LAC_PWM_TOP 125 //top value for PWM counter

/* GLOBAL VARIABLES */
uint8_t serial_position_setpoint = 0;
uint16_t MS5837_calibration_data[7];
uint8_t status = 0; //status byte: | | | | | |M/A|HD|HU|
uint32_t depth = 0;
int8_t angle = 0;

/*INTERRUPT SERVICE ROUTINES*/
//control loop interrupt routine, called at 1Hz
ISR(TIMER1_COMPA_vect){
  PORTB |= (1 << PB0);
  status = ((PINB&PB2) << 2) | ((PINC&PC1) << 1) | ((PINC&PC0) << 0);
  //if M/A bit high
  if((status & 0x04) == 0x04){ //automatic mode
    //TODO
  } else if ((status & 0x07) == 0x01){ //up
    OCR0B = 83;
    printString("Up");
  } else if ((status & 0x07) == 0x02){ //down
    OCR0B = 43;
    printString("Down");
  } else if((status & 0x07) == 0x03){ //neutral
    OCR0B = 63;
    printString("Neutral");
  } else {
    printString("Something went wrong :(");
  }
  _delay_ms(1);
  PORTB &= ~(1 << PB0);
}

/* INITIALIZATION FUNCTIONS */
//program start function for debug mode
void initDebug(void){
  if(DEBUG){
      DDRB |= (1<<PB0);
      PORTB |= (1 << PB0);
      _delay_ms(1000);
      PORTB &= ~(1 << PB0);
      _delay_ms(1000);
      PORTB |= (1 << PB0);
      initUSART();
      printString("\n\nSubmarine Controller\n");
      printString(VERSION);
      printString("\nPress any key:\n");
      receiveByte(); //wait to receive input
      printString("Starting Program\n");
      PORTB &= ~(1 << PB0);

  }
}
//initialize actuator control PWM on TIMER 0 at 1kHz
void initPWMTimer(void){
  DDRD |= (1 << PD5); //PWM output pin
  TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); //fast PWM mode
  TCCR0B = (1 << WGM02) | (1 << CS01) | (1 << CS00); //use OCRA as TOP to set frequency, prescaler F_CPU/64
  OCR0A = LAC_PWM_TOP; //set TOP value
  OCR0B = 63; //set duty cycle to zero (DC = OCR0B/OCROA*100%)
}
//initialize the ADC to read from the potentiometer
void initADC(void){
  DDRD |= (0 << PD7); //set up auto/manual control switch to input
  ADMUX |= (1 << ADLAR) | (1 << REFS0)| (1 << MUX1) | (1 << MUX0); //left adjust, AVcc reference, pin PC3/ADC3
  ADCSRA |= (1 << ADATE) | (1 << ADPS1) | (1 << ADPS0); //ADC prescale /8, free-run
  ADCSRA |= (1 << ADEN); //Enable
  ADCSRA |= (1 << ADSC); //start conversions
}
//initialize RPM counter interrupt
void initRPMCounter(void){
  cli();
  EIMSK |= (1 << INT0); //enable INT0
  EICRA |= (1 << ISC00); //edge detect
  sei();
}
//intialize contorl loop timer interrupt on TIMER 1
void initControlLoopTimer(void){
  cli();
  TCCR1B |= (1 << CS12) | (0 << CS10); //prescaler F_CPU/256
  TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode
  TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt
  OCR1A = 31294; // 1 Hz with 256 prescaler (double spec value, not sure why)
  sei();
}
//initialize manual control GPIO
void initGPIO(void){
  //set manual control inputs
  PORTC |= (1 << PC0) | (1 << PC1);
  PORTB |= (1 << PB2);

}

/* MAIN */
int main(void) {

  //initializations
  sei(); //  Enable global interrupts
  initDebug();
  initControlLoopTimer();
  initPWMTimer();
  initMS5837(MS5837_calibration_data);

  //free-running polling loop
  while (1) {
  }
  return 0;
}
