
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "USART.h"
#include "i2c.h"
#include "MS5837.h"

//Submarine Controller

/* LOGISTICAL PREAMBLE */

#define VERSION "v0.1"
#define DEBUG 0 //set debug mode, enabling serial communication
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
#define LAC_PWM_TOP 127 //top value for PWM counter

//control loop definitions
#define ERROR_SATURATION 1000
#define LOW_PASS_COEF 2
#define MAX_AUTO_ANGLE 40 //+/- midpoint
#define MANUAL_ANGLE 40

/* GLOBAL VARIABLES */
uint16_t MS5837_calibration_data[7];

uint8_t status = 0; //status byte: | | | | | |M/A|HD|HU|
uint8_t char_code = 0;

//controller variables
uint8_t kp = 10;
uint8_t ki = 0;
uint8_t kd = 0;
uint32_t depth = 500;
uint16_t setpoint = 200;
int16_t error = 0;
int16_t prev_error = 0;
uint8_t midpoint = 60;

/*INTERRUPT SERVICE ROUTINES*/
//control loop interrupt routine
ISR(TIMER1_COMPA_vect){
  PORTB |= (1 << PB0);
  //get switch status
  status = ((PINC&0x02) << 1) | ((PINB&0x04) >> 1) | (PINC&0x01);

  //low battery indicator
  if((ADCL | ADCH << 8) < 660){ // ~11V
    PORTD |= (1 << 4);
  } else {
    PORTD &= ~(1 << 4);
  }

  //get depth
  depth = read_MS5837_depth(MS5837_calibration_data);

  int32_t output = midpoint;

  //automatic/manual modes
  if((status & 0x04) == 0x04){ //automatic mode

    prev_error = error;
    error = depth - setpoint;

    //error saturation
    if(error > ERROR_SATURATION) error = ERROR_SATURATION;
    else if(error < -ERROR_SATURATION) error = -ERROR_SATURATION;

    //low pass filter (1/4 new input);
    error = (error + (prev_error * 3)) >> 2;

    int16_t p_term = ((error*kp) >> 7);
    int16_t d_term = (((error - prev_error)*kd) >> 7);
    //control loop
    output = midpoint + p_term + d_term;

    //set saturation angles
    output = fmin(output, midpoint + MAX_AUTO_ANGLE);
    output = fmax(output, midpoint - MAX_AUTO_ANGLE);

    //manual mode
  } else if ((status & 0x07) == 0x01){ //up
    output = fmin(midpoint + MANUAL_ANGLE, LAC_PWM_TOP);
  } else if ((status & 0x07) == 0x02){ //down
    output = fmax(midpoint - MANUAL_ANGLE, 0);
  } else if((status & 0x07) == 0x03){ //neutral
    output = midpoint;
  }

  OCR0B = output;

  PORTB &= ~(1 << PB0);
}

/* INITIALIZATION FUNCTIONS */
//program start function for debug mode
void initDebug(void){
    DDRB |= (1<<PB0);
    PORTB |= (1 << PB0);
    printString("\n\nSubmarine Controller\n");
    printString(VERSION);
    printString("\nPress any key:\n");
    receiveByte(); //wait to receive input
    printString("Starting Program\n");
    PORTB &= ~(1 << PB0);
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
  ADMUX |= (1 << REFS0)| (1 << MUX2) | (1 << MUX1); //left adjust, AVcc reference, ADC7
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
  // 20 Hz or 1 Hz with 256 prescaler (double spec value, not sure why)
  #if DEBUG
    OCR1A = 31280;
    cli();
  #else
    OCR1A = 3128;
    sei();
  #endif

}
//initialize manual control GPIO
void initGPIO(void){
  //set manual control inputs
  PORTC |= (1 << PC0) | (1 << PC1);
  PORTB |= (1 << PB2);

  //led outputs
  DDRD |= (1 << 4) | (1 << 3);
}

void printVoltage(){
  uint32_t ADC_val = ADCL | (ADCH << 8);
  uint16_t millivolts = (ADC_val*4262) >> 8;
  char str[30];
  sprintf(str, "Battery Voltage: %u.%1.uV\n", millivolts/1000, (millivolts % 1000)/100);
  printString(str);
}

void printDepth(){
  char str[20];
  sprintf(str, "Depth: %li cm\n", depth);
  printString(str);
}

void printControlVars(){
  char str[60];
  sprintf(str, "Control loop variables:\n  kp:%u\n  ki:%u\n  kd:%u\n", kp, ki, kd);
  printString(str);
}

void getControlVar(char str[3], uint8_t *k){
  printString(str);
  *k = getNumber();
}

/* MAIN */
int main(void) {

  //initializations
  #if DEBUG
    initDebug();
  #endif

  initUSART();
  initGPIO();
  initADC();
  initPWMTimer();
  initControlLoopTimer();
  initMS5837(MS5837_calibration_data);
  //free-running polling loop
  while (1) {
    char_code = receiveByte();
    switch(char_code){
      case 'b':
        printVoltage();
        break;
      case 'k':
        printControlVars();
        break;
      case 'P': //read kP
        getControlVar("kp=", &kp);
        break;
      case 'I': //read kI
        getControlVar("ki=", &ki);
        break;
      case 'D': //read kD
        getControlVar("kd=", &kd);
        break;
      case 'd':
        printDepth();
        break;
      case 'S':
        printString("Setpoint=");
        setpoint = getWord();
        break;
      case 'O':
        printString("Position=");
        OCR0B = getNumber();
        break;
      case 's': //stop/start
        if((SREG&0x80)==0x80){ //interrupts enabled
          cli();
          printString("stop\n");
        } else {
          printString("start\n");
          sei();

        }
        break;
      case 'm':
        printString("Set midpoint [50,70]");
        midpoint = getNumber();
        break;
      default:
        break;
    }
  }
  return 0;
}
