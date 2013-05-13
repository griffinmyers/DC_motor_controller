// Lab 4 for ECE 4760, Spring 2012, Cornell University
// Written by William Myers (wgm37) and Guo Jie Chin (gc348), Feb 26, 2012
// Port Description
// A[0:7] - Debug LEDs
// B[3] - PWM Output
// C[0:7] - LCD
// D[0:1] - UART
// D[2] - Fan Input


#define SEM_RX_ISR_SIGNAL 1			// rx isr
#define SEM_STRING_DONE 2 			// user hit <enter>
#define SEM_S 3						// speed variable
#define SEM_P 4						// P gain variable
#define SEM_I 5						// I gain variable 
#define SEM_D 6						// D gain variable 
#define SEM_RPM 7					// actual period value in RPM

#include "trtSettings.h"
#include "trtkernel644.c"
#include <util/delay.h>
#include <stdio.h>
#include "trtUart.h"
#include "trtUart.c"
#include <avr/sleep.h>
#include "lcd_lib.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <math.h>

//flash storage for LCD
const int8_t LCD_desired[] PROGMEM = "Desired:\0";
const int8_t LCD_current[] PROGMEM = "Current:\0";


// Program Functions
void initialize(void);										// some init procedures
void init_lcd(void);										// some LCD init procedures
float signum(float i);										// sign function

// Generally nice variables to have around
int8_t lcd_buffer[17];										// LCD output
int args[3];												// thread arguments; not used
const unsigned int prescalar = 32;							// prescalar for RPM calc
const long clock = 16000000;								// clock value
const float scale_factor = 4286000;							// rpm scale factor 60/(prescalar/clock) / 7
const float delta_t = 1/50;									// interrupt rate!
unsigned int global_s;										// global speed value
float global_p;												// p gain
float global_i;												// i gain
float global_d;												// d gain
float rpm;													// rpm value of motor	
volatile int motor_period;									// motor period [cycles]
volatile int motor_period_ovlf;								// overflow for 16 bit counting			

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// --------------------------------------------------------------------------------------
// Hardware interrupt every 1/7 period
ISR (INT0_vect) {
        motor_period = TCNT2 + motor_period_ovlf;	// calc period in cycles
        TCNT2 = 0;									// re-init
        motor_period_ovlf = 0;						// re-init
}
// Interupt on every overflow of Timer 2 - allows 16 bit counting
ISR (TIMER2_OVF_vect) {
        motor_period_ovlf = motor_period_ovlf + 256 ;
}

void get_and_set(void* args){

    uint16_t value;									// whatever value we're trying to set to
	char cmd[4];									// what command the user imputted
    
	while(1){
	
		fprintf(stdout, "\n\r>") ;					// Print a prompt for user input
		fscanf(stdin, "%s%d", cmd, &value) ;		// wait for the user input here
		trtWait(SEM_STRING_DONE);					// wait until enter has been pressed
		
		switch(cmd[0]){
			case 's':
				trtWait(SEM_S);
					global_s = value;
				trtSignal(SEM_S);
			break;
			case 'p':
				trtWait(SEM_P);
					global_p = (float)value;
				trtSignal(SEM_P);
			break;
			case 'i':
				trtWait(SEM_I);
					global_i = (float)value;
				trtSignal(SEM_I);
			break;
			case 'd':
				trtWait(SEM_D);
					global_d = (float)value/delta_t;		// factor in the delta_t differential
				trtSignal(SEM_D);
			break;	
		}					
	}
}

void pid_control(void* args){

  	float error;									// global_s - rpm
	float prev_error;								// for diff eq
	float error_sum;								// I sum
  	uint32_t rel, dead;								// some variables for scheduling
  	int motor_output;						// PWM output value
  	
  	//inits
  	global_s = 1000;
  	global_p = 5;
  	global_i = 3;
  	global_d = 30/delta_t;
  	rpm = 0;
  	prev_error = 500;
  	error_sum = 0;

	while(1){
	
		trtWait(SEM_RPM);										// wait until we've got control of the motor
		trtWait(SEM_S);											// wait until we've got control of s
			rpm = scale_factor/((float)motor_period);			// rpm calculation
			error = (float)global_s - rpm;						// error calc
		trtSignal(SEM_RPM);
		trtSignal(SEM_S);
			
		// detect a zero-crossing and restart the sum	
		if(signum(error) == signum(prev_error)){
			error_sum += error;
		}	
		else{
			error_sum = 0;
		}
				
		trtWait(SEM_P);
		trtWait(SEM_I);
		trtWait(SEM_D);
			motor_output = fmin(255,((int)global_p * (int)error) + ((int)global_d * ((int)error - (int)prev_error)) + (fmin(30, (int)global_i * (int)error_sum)));
		trtSignal(SEM_P);
		trtSignal(SEM_I);
		trtSignal(SEM_D);

		//fprintf(stdout,"%d\n\r",(int)error);
		
		
		if(motor_output > 0) {OCR0A = 255 - motor_output;}
		else {OCR0A = 255;}
		
		prev_error = error;					// store for next calc
		
		rel = trtCurrentTime() + SECONDS2TICKS(0.01);		
	    dead = trtCurrentTime() + SECONDS2TICKS(0.02);	
	    trtSleepUntil(rel, dead);	
	}

}


void update_LCD(void* args){

    uint32_t rel, dead;									// some variables for scheduling
    init_lcd();											// setup some LCD stuff once
	CopyStringtoLCD(LCD_desired,0,0);					// Desired:
    CopyStringtoLCD(LCD_current,0,1);					// Current:
    
	while(1){
		
		trtWait(SEM_S);
			dtostrf(global_s,1,3,lcd_buffer);			// Speed value
			LCDGotoXY(9,0);
			LCDstring(lcd_buffer, strlen(lcd_buffer));	
		trtSignal(SEM_S);
		
		trtWait(SEM_RPM);
			dtostrf(rpm,1,3,lcd_buffer);			// Speed value
			LCDGotoXY(9,1);
			LCDstring(lcd_buffer, strlen(lcd_buffer));		
		trtSignal(SEM_RPM);
		
		rel = trtCurrentTime() + SECONDS2TICKS(0.2);	// don't execute until 200ms from now
	    dead = trtCurrentTime() + SECONDS2TICKS(0.4);	// finish by 400ms from now
	    trtSleepUntil(rel, dead);					
	}
}

int main(void)
{
	initialize();

	set_sleep_mode(SLEEP_MODE_IDLE);
  	sleep_enable();
  	while (1) {
		sleep_cpu();
  	}
}

float signum(float i) {
  return (i>0)?1:((i<0)?-1:0);
}

void init_lcd(void){
	LCDinit();
	LCDcursorOFF();
	LCDclr();
}

void initialize(void)
{
	DDRB = 0xff;

  	//set up INT0 hardware driven interrupt and Timer 2 counting
	EIMSK = 1 << INT0 ; 						// turn on int0
	EICRA = 3 ;       							// rising edge
	TCCR2B = 3 ; 								// divide by 256, turn on timer 2 to be read in int0 ISR
	TIMSK2 = 1 ;								// turn on timer 2 overflow ISR for float precision time
	  	
  	 // set up timer 0 for PWM output
   	TCCR0B = 3 ;  			  	 				// prescalar = 64
   	TIMSK0 = 0 ;								// turn off timer 0 overflow ISR
   	TCCR0A = (1<<COM0A0) | (1<<COM0A1) | (1<<WGM00) | (1<<WGM01) ;	// turn on fast PWM, OC0A output, full clock, toggle OC0A, 16us per PWM cycle 
   	//OCR0A = 128; 
	
	//init the UART -- trt_uart_init() is in trtUart.c
  	trt_uart_init();
  	stdout = stdin = stderr = &uart_str;
  	fprintf(stdout,"Starting Lab 4...\n\r");
  	
								// set PWM to half full scale
  	
  	// start TRT
  	trtInitKernel(80); 							// 80 bytes for the idle task stack
  	
  	// create semphores
  	trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; 	// uart receive ISR semaphore
  	trtCreateSemaphore(SEM_STRING_DONE,0) ;  	// user typed <enter>
  	trtCreateSemaphore(SEM_S, 1) ; 				// protect shared variable
  	trtCreateSemaphore(SEM_P, 1) ; 				// protect shared variable
  	trtCreateSemaphore(SEM_I, 1) ; 				// protect shared variable
  	trtCreateSemaphore(SEM_D, 1) ; 				// protect shared variable
  	trtCreateSemaphore(SEM_RPM,1);				// protect shared variable
  
	
 	// create tasks
  	trtCreateTask(get_and_set, 200, SECONDS2TICKS(0.05), SECONDS2TICKS(0.05), &(args[0]));
  	trtCreateTask(pid_control, 200, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
  	trtCreateTask(update_LCD, 200, SECONDS2TICKS(0.1), SECONDS2TICKS(0.4), &(args[2]));
  	
}
