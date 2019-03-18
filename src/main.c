
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include "pinDefines.h"
#include <avr/interrupt.h>
#include "USART.h"
#include "i2c.h"
#include "MS5837.h"

//Submarine Controller

#define VERSION "v0.1"
#define DEBUG 1 //set debug mode, enabling serial communication
#define LAC_PWM_TOP 128 //top value for PWM counter

//globals
uint8_t serial_position_setpoint = 0;

//control loop interrupt routine
ISR(TIMER1_COMPA_vect){
   PORTB ^= (1 << 0); // Toggle the LED

  if((PIND && PD7)){ // <<< TODO: fix so this properly evaluates PD7
    OCR0B = (ADCH >> 1); //bit shift to reduce to 0-125
  } else {
    OCR0B = serial_position_setpoint;
  }


}

//initialize control loop timer1
void initControlLoopTimer(void){
  sei(); //  Enable global interrupts
  DDRB |= (1 << PB0); // Set LED as output
  TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode
  OCR1A   = 15624; // Set output compare value
  TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt
  TCCR1B |= ((1 << CS10) | (1 << CS11)); //prescaler F_CPU/64
}

//initialize PWM on PD5 to control actuator using timer0
void initPWMTimer(void){
  DDRD |= (1 << PD5); //PWM output pin
  TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); //fast PWM mode
  TCCR0B = (1 << WGM02) | (1 << CS01); //use OCRA as TOP to set frequency, prescaler F_CPU/8
  OCR0A = LAC_PWM_TOP; //set the PWM frequency
  OCR0B = 0; //set duty cycle = OCR0B/OCR0A * 100%
}

void initDirectControlPot(void){
  DDRD |= (0 << PD7); //set up auto/manual control switch to input
  ADMUX |= (1 << ADLAR) | (1 << REFS0)| (1 << MUX1) | (1 << MUX0); //left adjust, AVcc reference, pin PC3/ADC3
  ADCSRA |= (1 << ADATE) | (1 << ADPS1) | (1 << ADPS0); //ADC prescale /8, free-run
  ADCSRA |= (1 << ADEN); //Enable
  ADCSRA |= (1 << ADSC); //start conversions
}

int main(void) {

  //set up serial communications for debug mode
  if(DEBUG){
      initUSART();
      printString("\n\nSubmarine Controller\n");
      printString(VERSION);
      printString("\nPress any key:\n");
      receiveByte(); //wait to receive input
      printString("Starting Program\n");
  }

  // General initialization stuff
  initControlLoopTimer();
  initPWMTimer();
  //initMS5837();
  initDirectControlPot();

  while (1) {
    serial_position_setpoint = getNumber()*1.28;
  }
  return 0;
}
