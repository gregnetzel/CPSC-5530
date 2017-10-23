//*****************************************************************************
//
// Built from hello.c - Simple hello world example.
//
// Copyright (c) 2006-2011 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 7243 of the EK-LM3S8962 Firmware Package.
//
//*****************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "inc/lm3s8962.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "drivers/rit128x96x4.h"
<<<<<<< HEAD
#include  <utils/uartstdio.c>
=======
#include "driverlib/pwm.h"
>>>>>>> c87cfdca997f4939cf8fd671ed1a20ac27626666

#define TRUE 1                               //used for display to OLED
#define FALSE 0
#define hSpacing 8
#define vSpacing 14
#define NUMTASKS 8
volatile int i = 0;
volatile unsigned long ulLoop;
volatile unsigned long time = 0; //in tenths of seconds

//Enum
enum displayMode { MENU_HOVER = 0, ANNUN_HOVER = 1, 
                   HR_HOVER = 2, BP_HOVER = 3, 
                  TEMP_HOVER = 4, ANNUNCIATE = 5, 
                  HR = 6, BP = 7, TEMP = 8};
typedef enum displayMode displayMode;

//Data Variables
unsigned int tempRawBuff[8];
unsigned int bloodPressRawBuff[16];	//sys in first half, dia in second.
unsigned int pulseRateRawBuff[8];

float tempCorrectedBuff[8];
unsigned int bloodPressCorrectedBuff[16];//sys in first half, dia in second
unsigned int pulseRateCorrectedBuff[8];

// Keypad additions
unsigned short mode = 0;
unsigned short measurementSelection = 0;
unsigned short scroll = 0;
unsigned short select = 0;
unsigned short alarmAcknowledge = 0;

unsigned short batteryState = 200;

unsigned char bpOOR = '0';
unsigned char pressOOR = '0';
unsigned char tempOOR = '0';

int bpHigh = FALSE;
int tempHigh = FALSE;
int pulseLow = FALSE;

//Structs
typedef struct MyStruct TCB;
struct MyStruct {
	void(*myTask)(void*);
	void* taskDataPtr;
	TCB* next;
	TCB* prev;
}; 

struct LinkedList {  
	TCB* head;
	TCB* tail;
}; typedef struct LinkedList LinkedList;

struct Measurements {
	unsigned int* tempRawBuff;
	unsigned int* bloodPressRawBuff;
	unsigned int* pulseRateRawBuff;
	unsigned short* measurementSelection;
	int reverseTemp;
	int sysComplete;
	int reversePulse;
}; typedef struct Measurements Measurements;

struct ComputeData {
	unsigned int* tempRawBuff;
	unsigned int* bloodPressRawBuff;
	unsigned int* pulseRateRawBuff;
	float* tempCorrectedBuff;
	unsigned int* bloodPressCorrectedBuff;
	unsigned int* pulseRateCorrectedBuff;
	unsigned short* measurementSelection;
}; typedef struct ComputeData ComputeData;

struct Display {
	float* tempCorrectedBuff;
	unsigned int* bloodPressCorrectedBuff;
	unsigned int* pulseRateCorrectedBuff;
	unsigned short* batteryState;
	unsigned short* mode;
}; typedef struct Display Display;

struct Status {
	unsigned short* batteryState;
}; typedef struct Status Status;

struct WarningAlarm {
	unsigned int* tempRawBuff;
	unsigned int* bloodPressRawBuff;
	unsigned int* pulseRateRawBuff;
	unsigned short* batteryState;
}; typedef struct WarningAlarm WarningAlarm;

struct Keypad {
	unsigned short* mode;
	unsigned short* measurementSelection;
	unsigned short* scroll;
	unsigned short* select;
	unsigned short* alarmAcknowledge;
}; typedef struct Keypad Keypad;

struct Communications {
	float *tempCorrectedBuff;
	unsigned int* bloodPressCorrectedBuff;
	unsigned int* pulseRateCorrectedBuff;
        unsigned short* batteryState;
}; typedef struct Communications Communications;

//flags
volatile int upPressed = 0;
volatile int downPressed = 0;
volatile int leftPressed = 0;
volatile int selectPressed = 0;
volatile int addFlags[] = {0,0,0,0,0,0,0,0}; //what schedule needs to add, same order as tasks array

//interrupts
void selectPressedHandler(void){//port F pin 1
  GPIOPinIntClear(GPIO_PORTF_BASE, GPIO_PIN_1);
  selectPressed = 1;
}
void dirPressedHandler(void){//port E pins 0-3 up down left right
  volatile long testUp = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0);
  volatile long testDown = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1);
  volatile long testLeft = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2);
  if(testDown == 0){
    downPressed = 1;
  }
  else if(testUp == 0){
    upPressed = 1;
  }
  else if(testLeft == 0){
    leftPressed = 1;
  }
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);
}

//functions
void delay(unsigned long aValue);
void print(char* c, int hOffset, int vOffset);
void measure(void* data);
void compute(void* data);
void display(void* data);
void annunciate(void* data);
void status(void* data);
void schedule(void* data);
void fillStructs(Measurements* m, ComputeData* c, Display* d, Status* s,
	WarningAlarm* w, Keypad* k, Communications* z);
void fillBuffers();
void intPrint(int c, int size, int hOffset, int vOffset);
void fPrint(float c, int size, int hOffset, int vOffset);
void llEnqueue(LinkedList* ll, TCB* task);
TCB* llDequeue(LinkedList* ll);
//*****************************************************************************
//
// Simulated Medical Device
//
//*****************************************************************************

LinkedList taskQueue;
TCB* tasks;

int main(void)
{
        SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);
        SysTickPeriodSet(SysCtlClockGet());
        SysTickEnable();
        
	//intitial data
	Measurements measurementData;
	ComputeData computeData;
	Display displayData;
	Status statusData;
	WarningAlarm warningAlarmData;
	Keypad keypadData;
	Communications communicationsData;
        
        tasks = malloc(NUMTASKS*sizeof(TCB));   //array to hold tasks when not in queue
        tasks[0].myTask = measure;
	tasks[1].myTask = compute;
        tasks[2].myTask = display;
	tasks[3].myTask = annunciate;
	tasks[4].myTask = status;
	tasks[5].myTask = schedule;
	tasks[0].taskDataPtr = &measurementData;
	tasks[1].taskDataPtr = &computeData;
	tasks[2].taskDataPtr = &displayData;
	tasks[3].taskDataPtr = &warningAlarmData;
	tasks[4].taskDataPtr = &statusData;
	tasks[5].taskDataPtr = NULL;
        
        TCB* activeTask;
        fillBuffers();
	RIT128x96x4Init(1000000); 
        
        //light enable
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;  // Enable the GPIO port that is used for the on-board LED.
	ulLoop = SYSCTL_RCGC2_R;              // Do a dummy read to insert a few cycles after enabling the peripheral.
	GPIO_PORTF_DIR_R = 0x01;              // Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
	GPIO_PORTF_DEN_R = 0x01;              // enable the GPIO pin for digital function.
        
        //uart enable for serial comm
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1);
	IntEnable(INT_UART0);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8|UART_CONFIG_PAR_NONE|UART_CONFIG_STOP_ONE);
        
        //up down buttons enabled
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
        GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
        GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_FALLING_EDGE);
        GPIOPinIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
        IntEnable(INT_GPIOE);
        
        //select button (for some reason on same port as light which is annoying)
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
        GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
        GPIOPinIntEnable(GPIO_PORTF_BASE, GPIO_PIN_1);
        IntEnable(INT_GPIOF);

        // speaker
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
        GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
        GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
        unsigned long ulPeriod = SysCtlClockGet() / 440;
        PWMGenConfigure(PWM_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
        PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod);
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, ulPeriod / 4);
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ulPeriod * 3 / 4);
        
	fillStructs(&measurementData, &computeData, &displayData, &statusData,
		&warningAlarmData, &keypadData, &communicationsData);
        
	while (TRUE)
	{
            tasks[5].myTask(tasks[5].taskDataPtr);//Schedule task
            activeTask = llDequeue(&taskQueue);
            while(activeTask != NULL){//queue isn't empty
              activeTask->myTask(activeTask->taskDataPtr);
              activeTask = llDequeue(&taskQueue);
            }
            tasks[2].myTask(tasks[2].taskDataPtr);//Display
	}
}
<<<<<<< HEAD

//Startup, Measure, Compute, Display, Annunciate, Warning and Alarm, Status, SerialCommunications, and Schedule

void SerialCommunications(void* data){
  Communications* d = data;
  char buffer[50];
  /* float *tempCorrectedBuff;
	unsigned int* bloodPressCorrectedBuff;
	unsigned int* pulseRateCorrectedBuff;*/
  sprintf(buffer,"1.Temperature:  %d\n2. Systolic pressure: %immHG\n3. Dyastolic pressure: %immHG\n4. Pulse rate: %iBPM\n5. Battery: %i",
          d->tempCorrectedBuff[0],d->bloodPressCorrectedBuff[0],d->bloodPressCorrectedBuff[8],pulseRateCorrectedBuff[0],d->batteryState);
  
  char* buf = &buffer[0];
  while(UARTBusy(UART0_BASE));
  while(*buf != '\0'){
    UARTCharPut(UART0_BASE, *buf++);
  }
}

void measure(void* data) {/*
=======
//Startup, Measure, Compute, Display, Annunciate, Warning and Alarm, Status, Local Communications, and Schedule
void measure(void* data) {
>>>>>>> c87cfdca997f4939cf8fd671ed1a20ac27626666
	//temperature
	if (((Measurements*)data)->reverseTemp == FALSE) {    //increasing pattern
		if (((Measurements*)data)->tempRawBuff[0] > 50) {
			((Measurements*)data)->reverseTemp = TRUE;      //reverse pattern
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->tempRawBuff[0] -= 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->tempRawBuff[0] += 1;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->tempRawBuff[0] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->tempRawBuff[0] -= 1;
			}
		}
	}
	else {                                              //decreasing pattern
		if (*((Measurements*)data)->tempRawBuff < 15) {
			((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->tempRawBuff[0] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->tempRawBuff[0] -= 1;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->tempRawBuff[0] -= 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->tempRawBuff[0] += 1;
			}
		}
	}
	//systolic/diastolic pressure
	if (((Measurements*)data)->sysComplete == FALSE) {   //run systolic
		if (((Measurements*)data)->bloodPressRawBuff[0] > 100) {      //systolic complete
			((Measurements*)data)->sysComplete = TRUE;      //run diatolic
			((Measurements*)data)->bloodPressRawBuff[0] = 80;          //rest systolic
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->bloodPressRawBuff[8] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->bloodPressRawBuff[8] -= 1;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->bloodPressRawBuff[0] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->bloodPressRawBuff[0] -= 1;
			}
		}
	}
	else {                                              //run diatolic
		if (((Measurements*)data)->bloodPressRawBuff[0] < 40) {       //diatolic complete
			((Measurements*)data)->sysComplete = FALSE;     //run systolic
			((Measurements*)data)->bloodPressRawBuff[8] = 80;          //reset diatolic
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->bloodPressRawBuff[0] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->bloodPressRawBuff[0] -= 1;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->bloodPressRawBuff[8] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->bloodPressRawBuff[8] -= 1;
			}
		}
	}

	//heartrate
	if (((Measurements*)data)->reversePulse == FALSE) {   //increasing pattern
		if (((Measurements*)data)->pulseRateRawBuff[0] > 40) {
			((Measurements*)data)->reversePulse = TRUE;     //reverse pattern
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->pulseRateRawBuff[0] += 1;
			}
			else {                                          //odd tick
				((Measurements*)data)->pulseRateRawBuff[0] -= 3;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->pulseRateRawBuff[0] -= 1;
			}
			else {                                          //odd tick
				((Measurements*)data)->pulseRateRawBuff[0] += 3;
			}
		}
	}
	else {                                              //decreasing pattern
		if (((Measurements*)data)->pulseRateRawBuff[0] < 15) {
			((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->pulseRateRawBuff[0] -= 1;
			}
			else {                                          //odd tick
				((Measurements*)data)->pulseRateRawBuff[0] += 3;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				((Measurements*)data)->pulseRateRawBuff[0] += 1;
			}
			else {                                          //odd tick
				((Measurements*)data)->pulseRateRawBuff[0] -= 3;
			}
		}
	}

}

void compute(void* data) {
	float t = (float)(((ComputeData*)data)->tempRawBuff[0]);
	unsigned int s = (((ComputeData*)data)->bloodPressRawBuff[0]);
	unsigned int d = (((ComputeData*)data)->bloodPressRawBuff[0]); 
	unsigned int h = (((ComputeData*)data)->pulseRateRawBuff[0]);

	t = (5 + (0.75*t));
	s = 9 + (2 * s);
	d = (int)(6 + (1.5*d));
	h = 8 + (3 * h);
	((ComputeData*)data)->tempCorrectedBuff[0] = t;
	((ComputeData*)data)->bloodPressCorrectedBuff[0] = s;
	((ComputeData*)data)->bloodPressCorrectedBuff[0] = d;
	((ComputeData*)data)->pulseRateCorrectedBuff[0] = h;
        
        //maybe have a function to increment all pointers.
}

/*MENU_HOVER = 0, ANNUN_HOVER = 1,
HR_HOVER = 2, BP_HOVER = 3, 
TEMP_HOVER = 4, ANNUNCIATE = 5, 
HR = 6, BP = 7, TEMP = 8*/

void display(void* data) {
  unsigned short* m = ((Display*)data)->mode;
  if (selectPressed == 1){
    selectPressed = 0;
    upPressed = 0;
    downPressed = 0;
    RIT128x96x4Clear();
    
    switch(*m) {
    case 0:
      *m = 2;
      break;
    case 1:
      *m = 5;
      break;
    case 2:
      *m = 6;
      break;
    case 3:
      *m = 7;
      break;
    case 4:
      *m = 8;
      break;
    }
  }
  else{
    if (upPressed == 1){
      RIT128x96x4Clear();
      switch(*m) {
      case 0:
        break;
      case 1:
        *m = 0;
        break;
      case 2:
        break;
      case 3:
        *m = 2;
        break;
      case 4:
        *m = 3;
        break;
      }
      upPressed = 0;
    }
    if (downPressed == 1){
      RIT128x96x4Clear();
      switch(*m) {
      case 0:
        *m = 1;
        break;
      case 1:
        break;
      case 2:
        *m = 3;
        break;
      case 3:
        *m = 4;
        break;
      case 4:
        break;
      }
      downPressed = 0;
    }
    if (leftPressed == 1){
      RIT128x96x4Clear();
      switch(*m) {
      case 2:
      case 3:
      case 4:
        *m = 0;
        break;
      case 5:
        *m = 1;
        break;
      case 6:
        *m = 2;
        break;
      case 7:
        *m = 3;
        break;
      case 8:
        *m = 4;
        break;
      }
      leftPressed = 0;
    }
  }
  if( *m == MENU_HOVER || *m == ANNUN_HOVER ){  // mode selection
    print("Please select a Mode:", 0, 0);
  
    if( *m == MENU_HOVER){//  menu mode hover
      print("* Menu", 0, 1);
    }else{
      print("  Menu", 0, 1);
    }
  
    if( *m == ANNUN_HOVER){//  annunciate hover
      print("* Annunciate", 0, 2);
    }else{
      print("  Annunciate", 0, 2);
    }
  }
   
  if( *m == BP_HOVER || *m == TEMP_HOVER || *m == HR_HOVER){  // Menu Mode
    print("Menu", 0, 0);
  
    if( *m == HR_HOVER ){//  Heart Rate hover
      print("* Heart Rate", 0, 1);
    }else{
      print("  Heart Rate", 0, 1);
    }
  
    if( *m == BP_HOVER){    //  Blood Pressure hover
      print("* Blood Pressure", 0, 3);
    }else{
      print("  Blood Pressure", 0, 3);
    }
  
    if( *m == TEMP_HOVER ){    //  Temperature hover
      print("* Temperature", 0, 5);
    }else{
      print("  Temperature", 0, 5);
    }
  }
  
  if( *m == BP || *m == HR || *m == TEMP){  // Menu option display
    print("Menu", 0, 0);
    
    if( *m == BP){  // Blood Pressure
      print("Blood Pressure", 0, 1);
      volatile int t = *((Display*)data)->bloodPressCorrectedBuff;
      intPrint(((Display*)data)->bloodPressCorrectedBuff[0], 3, 0, 2);         //Systolic: should never be over 3 char
      print("/", 3, 2);
      intPrint(((Display*)data)->bloodPressCorrectedBuff[7], 5, 4, 2);         //Diastolic: should never be over 5 char
      print("mm Hg", 9, 2);
    }
  
    if( *m == TEMP){  // temperature
      print("Temperature:", 0, 1);
      fPrint(((Display*)data)->tempCorrectedBuff[0], 4, 0, 2);               //Temperature: should never be over 4 char
      print("C", 4, 2);
    }
    
    if( *m == HR){  // heart rate
      print("Heart Rate:", 0, 1);
      intPrint(((Display*)data)->pulseRateCorrectedBuff[0], 3, 0, 2);        //Heartrate: should never be over 3 char
      print("BPM", 3, 2);
    }
  }
   
  if( *m == ANNUNCIATE){  // Annunciate Mode
    print("Annunciate:", 0, 0);
    volatile int t = *((Display*)data)->bloodPressCorrectedBuff;
    intPrint(((Display*)data)->bloodPressCorrectedBuff[0], 3, 0, 1);         //Systolic: should never be over 3 char
    print("/", 3, 1);
    intPrint(((Display*)data)->bloodPressCorrectedBuff[7], 5, 4, 1);         //Diastolic: should never be over 5 char
    print("mm Hg", 9, 1);

    fPrint(((Display*)data)->tempCorrectedBuff[0], 4, 0, 2);               //Temperature: should never be over 4 char
    print("C", 4, 2);
    intPrint(((Display*)data)->pulseRateCorrectedBuff[0], 3, 6, 2);        //Heartrate: should never be over 3 char
    print("BPM", 9, 2);

    intPrint(*((Display*)data)->batteryState, 3, 13, 2);    //battery: should never be over 3 char
  }

}

void annunciate(void* data) {/*

  PWMGenEnable(PWM_BASE, PWM_GEN_0);
  PWMGenDisable(PWM_BASE, PWM_GEN_0);
  
	float t = (float)*(((WarningAlarm*)data)->temp);
	unsigned int s = *(((WarningAlarm*)data)->sysPress);
	float d = (float)*(((WarningAlarm*)data)->diaPress);
	unsigned int h = *(((WarningAlarm*)data)->heartRate);
	short b = *(((WarningAlarm*)data)->batteryState);
	int counter = 0;                                //for "alarm cycle"

	t = 5 + (0.75*t);
	s = 9 + (2 * s);
	d = 6 + (1.5*d);
	h = 8 + (3 * h);

	if (b < 40) {
		while (counter < 3) {                          //flash light at 3 second interval
			GPIO_PORTF_DATA_R &= ~(0x01);               // Turn off the LED
			delay(30);
			GPIO_PORTF_DATA_R |= 0x01;                  // Turn on the LED
			delay(30);
			counter++;
		}
	}

	if (t > 37.8 || t < 36.1) {
		while (counter < 3) {                           //flash light at 1 second interval
			GPIO_PORTF_DATA_R &= ~(0x01);               // Turn off the LED
			delay(20);
			GPIO_PORTF_DATA_R |= 0x01;                  // Turn on the LED
			delay(20);
			counter++;
		}
	}

	if (h > 100 || h < 60) {
		while (counter < 3) {                           //flash light at 2 second interval
			GPIO_PORTF_DATA_R &= ~(0x01);               // Turn off the LED
			delay(40);
			GPIO_PORTF_DATA_R |= 0x01;                  // Turn on the LED
			delay(40);
			counter++;
		}
	}

	if (s > 120 || s < 90 || d > 80 || d < 60) {
		while (counter < 3) {                           //flash light at 0.5 second interval
			GPIO_PORTF_DATA_R &= ~(0x01);               // Turn off the LED
			delay(10);
			GPIO_PORTF_DATA_R |= 0x01;                  // Turn on the LED
			delay(10);
			counter++;
		}
	}

	GPIO_PORTF_DATA_R |= 0x01;                     // Turn on the LED for normal state.
*/
}


void status(void* data) {

	*(((Status*)data)->batteryState) -= 1;              //decrement battery by 1
}

void schedule(void* data) {
	delay(1);
	i++;
        time++;
        for (int j = 0; j < NUMTASKS; j++){
          if(addFlags[j] != 0){
            addFlags[j] = 0;
            llEnqueue(&taskQueue,&tasks[j]);
          }
        }
}

void delay(unsigned long aValue) {
    while(aValue--){
        while(SysTickValueGet() > 1000){
        }
        while(SysTickValueGet() < 1000){
        }
    }
}

void print(char* c, int hOffset, int vOffset) {                        // string, column, row
	RIT128x96x4StringDraw(c, hSpacing*(hOffset), vSpacing*(vOffset), 15);
}

void intPrint(int c, int size, int hOffset, int vOffset) {             // number, size of number,column, row
	char dec[2];
	dec[1] = '\0';
	int rem = c;
	for (int i = size - 1; i >= 0; i--) {
		dec[0] = rem % 10 + '0';
		rem = rem / 10;
		RIT128x96x4StringDraw(dec, hSpacing*(hOffset + i), vSpacing*(vOffset), 15);
	}
}
//print of a float with one decimal
void fPrint(float c, int size, int hOffset, int vOffset) {// number, size of number,column, row 
	char dec[2];
	dec[1] = '\0';
	int rem = (int)c * 10;
	for (int i = size - 1; i >= 0; i--) {
		if (i == size - 2)
			RIT128x96x4StringDraw(".", hSpacing*(hOffset + i), vSpacing*(vOffset), 15);
		else {
			dec[0] = rem % 10 + '0';
			rem = rem / 10;
			RIT128x96x4StringDraw(dec, hSpacing*(hOffset + i), vSpacing*(vOffset), 15);
		}
	}
}

void fillStructs(Measurements* m, ComputeData* c, Display* d, Status* s, WarningAlarm* w, Keypad* k, Communications* z) {

	m->tempRawBuff = &tempRawBuff[0];                   //measure
	m->bloodPressRawBuff = &bloodPressRawBuff[0];
	m->pulseRateRawBuff = &pulseRateRawBuff[0];
	m->reverseTemp = FALSE;
	m->sysComplete = FALSE;
	m->reversePulse = FALSE;

	c->tempRawBuff = &tempRawBuff[0];                            //compute
	c->bloodPressRawBuff = &bloodPressRawBuff[0];
	c->pulseRateRawBuff = &pulseRateRawBuff[0];
	c->tempCorrectedBuff = &tempCorrectedBuff[0];
	c->bloodPressCorrectedBuff = &bloodPressCorrectedBuff[0];
	c->pulseRateCorrectedBuff = &pulseRateCorrectedBuff[0];

	d->tempCorrectedBuff = &tempCorrectedBuff[0];                //display
	d->bloodPressCorrectedBuff = &bloodPressCorrectedBuff[0];
	d->pulseRateCorrectedBuff = &pulseRateCorrectedBuff[0];
	d->batteryState = &batteryState;
	d->mode = &mode;

	s->batteryState = &batteryState;                    //status

	w->tempRawBuff = &tempRawBuff[0];                   //warningAlarm
	w->bloodPressRawBuff = &bloodPressRawBuff[0];
	w->pulseRateRawBuff = &pulseRateRawBuff[0];
	w->batteryState = &batteryState;

	k->mode = &mode;
	k->measurementSelection = &measurementSelection;
	k->scroll = &scroll;
	k->select = &select;
	k->alarmAcknowledge = &alarmAcknowledge;

	z->tempCorrectedBuff = &tempCorrectedBuff[0];
	z->bloodPressCorrectedBuff = &bloodPressCorrectedBuff[0];
	z->pulseRateCorrectedBuff = &pulseRateCorrectedBuff[0];
        z->batteryState  = &batteryState;
}

void fillBuffers() {
  for(int i = 0; i < 8; i++) {
    tempRawBuff[i] = 75;
    bloodPressRawBuff[i] = 80;
    bloodPressRawBuff[i+7] = 80;
    pulseRateRawBuff[i] = 0;
  }
}
void llEnqueue(LinkedList* ll, TCB* task){
	if (NULL == ll->head){
  		ll->head = task;
		ll->tail = task;
  	}
	else {
		ll->tail->next = task;
		task->prev = ll->tail;
		ll->tail = task;
	}
}

TCB* llDequeue(LinkedList* ll){
  	if( ll->head == NULL  )
    		return NULL;
	else{
	  	TCB* task = ll->head;
		ll->head = task->next;
		ll->head->prev = NULL;
		task->prev = NULL;
		task->next = NULL;
	  	return task;
	}
}