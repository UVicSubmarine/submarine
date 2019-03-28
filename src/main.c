
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



/*INTERRUPT SERVICE ROUTINES*/

//control loop interrupt routine, must complete in <65ms
ISR(TIMER2_COMPA_vect){
  PORTB ^= (1 << PB0);
/*
  if((PIND && PD7)){ // <<< TODO: fix so this properly evaluates PD7
    OCR0B = (ADCH >> 1); //bit shift to reduce to 0-125
  } else {
    OCR0B = serial_position_setpoint;
  }
  */
}

//RPM counter interrupt
ISR(INT0_vect){
  printString("Counter Value: ");
  uint16_t counter_value = (TCNT1H << 8) | (TCNT1L);
  TCNT1H = 0;
  TCNT1L = 0;
  printWord(counter_value);
  printString("\n");
}

/* INITIALIZATION FUNCTIONS */

//program start function for debug mode
void initDebug(void){
  if(DEBUG){
      DDRB |= (1<<PB0);
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

//initialize control loop interrupt on TIMER 2
void initControlLoopTimer(void){
  cli();
  TCCR2B |= (1 << WGM22); // Configure timer 1 for CTC mode
  OCR2A   = 255; // Set output compare value
  TIMSK2 |= (1 << OCIE2A); // Enable CTC interrupt
  TCCR2B |= (1 << CS22) | (1 << CS21) |(1 <<CS20); //prescaler F_CPU/1024
  sei();
}
//initialize actuator control PWM on TIMER 0 at 1kHz
void initPWMTimer(void){
  DDRD |= (1 << PD5); //PWM output pin
  TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); //fast PWM mode
  TCCR0B = (1 << WGM02) | (1 << CS01) | (1 << CS00); //use OCRA as TOP to set frequency, prescaler F_CPU/64
  OCR0A = LAC_PWM_TOP; //set TOP value
  OCR0B = 2; //set duty cycle to zero (DC = OCR0B/OCROA*100%)
}
//initialize the ADC to read from the potentiometer
void initDirectControlPot(void){
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
//intialize RPM counter timer on TIMER 1
void initRPMTimer(void){
  TCCR1B = (1 << CS11) | (1 << CS10); //prescaler F_CPU/
  TCCR1B |= (1 << WGM12) | (1 << WGM12); // Configure timer 1 for CTC mode
}

/* MAIN */

int main(void) {

  //initializations
  sei(); //  Enable global interrupts
  initDebug();
  initControlLoopTimer();
  initPWMTimer();
  initMS5837(MS5837_calibration_data);
  //initDirectControlPot();
  //initRPMTimer();
  //initRPMCounter();


  //free-running polling loop
  while (1) {
    serial_position_setpoint = getNumber()*1.28;
  }
  return 0;
}
