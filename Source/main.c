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
#include  <utils/uartstdio.c>
#include "driverlib/pwm.h"
#include "driverlib/timer.h"

#define TRUE 1                               //used for display to OLED
#define FALSE 0
#define hSpacing 8
#define vSpacing 14
#define NUMTASKS 8
volatile int i = 0;
volatile int j = 0;               // circular buffer usage
volatile int indexToDel = 0;
volatile unsigned long ulLoop;
volatile unsigned long time = 0; //in tenths of seconds
volatile unsigned long pulseTime = 0; //pulse transducer time

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
int speakerOn = FALSE;
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
volatile int addFlags[] = {1,0,0,0,0,0,0,0}; //what schedule needs to add, same order as tasks array
volatile int measureDelete = 0;

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
void timerHandler(void){//timer0 subtimerA
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Clear the timer interrupt.
    i++;                                            // increment system time
    time++;
    IntMasterDisable();                             // Disable and re-enable interrupt.
    IntMasterEnable();
}

void pulseTimerHandler(void) {// timer1 subtimerA
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // Clear the timer interrupt.
    pulseTime++;                                    // increment pulse time
    IntMasterDisable();                             // Disable and re-enable interrupt.
    IntMasterEnable();
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
void resetMeasureBufferAt(int j);
void intPrint(int c, int size, int hOffset, int vOffset);
void fPrint(float c, int size, int hOffset, int vOffset);
void llEnqueue(LinkedList* ll, TCB* task);
TCB* llDequeue(LinkedList* ll);
void serialCommunications(void* data);
//*****************************************************************************
//
// Simulated Medical Device
//
//*****************************************************************************

LinkedList* taskQueue;
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
        taskQueue = malloc(sizeof(LinkedList));
        taskQueue->head = NULL;
        taskQueue->tail = NULL;
        taskQueue->head->next = NULL;
        taskQueue->head->prev = NULL;
        
        tasks = malloc(NUMTASKS*sizeof(TCB));   //array to hold tasks when not in queue
        tasks[0].myTask = measure;
	tasks[1].myTask = compute;
        tasks[2].myTask = display;
	tasks[3].myTask = annunciate;
	tasks[4].myTask = status;
	tasks[5].myTask = schedule;
	tasks[6].myTask = serialCommunications;
	tasks[0].taskDataPtr = &measurementData;
	tasks[1].taskDataPtr = &computeData;
	tasks[2].taskDataPtr = &displayData;
	tasks[3].taskDataPtr = &warningAlarmData;
	tasks[4].taskDataPtr = &statusData;
	tasks[5].taskDataPtr = NULL;
        tasks[6].taskDataPtr = &communicationsData;
        for(int j = 0; j < NUMTASKS; j++){//set all the tasks to default pointing at null
          tasks[j].next = NULL;
          tasks[j].prev = NULL;
        }
        
        TCB* activeTask;
        fillBuffers();
	RIT128x96x4Init(1000000); 
        
                                              //light enable
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;  // Enable the GPIO port that is used for the on-board LED.
	ulLoop = SYSCTL_RCGC2_R;              // Do a dummy read to insert a few cycles after enabling the peripheral.
	GPIO_PORTF_DIR_R = 0x01;              // Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
	GPIO_PORTF_DEN_R = 0x01;              // enable the GPIO pin for digital function.
                                                           
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);  //uart enable for serial comm
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1);
	IntEnable(INT_UART0);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8|UART_CONFIG_PAR_NONE|UART_CONFIG_STOP_ONE);
        
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);          //up down buttons enabled
        GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
        GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
        GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_FALLING_EDGE);
        GPIOPinIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
        IntEnable(INT_GPIOE);
        
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  //select button (for some reason on same port as light which is annoying)
        GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
        GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
        GPIOPinIntEnable(GPIO_PORTF_BASE, GPIO_PIN_1);
        IntEnable(INT_GPIOF);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);    // speaker
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
        
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // global timer
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // pulse timer
        IntMasterEnable();
        TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
        TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
	TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/10); // need to play with how often timer ISR is run I think right now its 100ms.
        TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()*2);  // pulse transducer increments once every 2 seconds.
        IntEnable(INT_TIMER0A);
        IntEnable(INT_TIMER1A);
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
        TimerEnable(TIMER0_BASE, TIMER_A);
        TimerEnable(TIMER1_BASE, TIMER_A);
        
        fillStructs(&measurementData, &computeData, &displayData, &statusData,
		&warningAlarmData, &keypadData, &communicationsData);
        
	while (TRUE)
	{
            tasks[5].myTask(tasks[5].taskDataPtr);//Schedule task
            activeTask = llDequeue(taskQueue);
            while(activeTask != NULL){//queue isn't empty
              activeTask->myTask(activeTask->taskDataPtr);
              activeTask = llDequeue(taskQueue);
            }            
            tasks[2].myTask(tasks[2].taskDataPtr);//Display
	}
}

//Startup, Measure, Compute, Display, Annunciate, Warning and Alarm, Status, SerialCommunications, and Schedule

void serialCommunications(void* data){
  Communications* d = data;
  char buffer[50];
  /* float *tempCorrectedBuff;
	unsigned int* bloodPressCorrectedBuff;
	unsigned int* pulseRateCorrectedBuff;*/
  sprintf(buffer,"1.Temperature:  %d\n2. Systolic pressure: %dmmHG\n3. Dyastolic pressure: %dmmHG\n4. Pulse rate: %dBPM\n5. Battery: %d",
          d->tempCorrectedBuff[0],d->bloodPressCorrectedBuff[0],d->bloodPressCorrectedBuff[8],pulseRateCorrectedBuff[0],d->batteryState);
  
  char* buf = &buffer[0];
  while(UARTBusy(UART0_BASE));
  while(*buf != '\0'){
    UARTCharPut(UART0_BASE, *buf++);
  }
}

void measure(void* data) {
  int ind = i/10;
  if(i%10 == 0){                                          //temperature
	if (((Measurements*)data)->reverseTemp == FALSE) {    //increasing pattern
		if (((Measurements*)data)->tempRawBuff[j] > 50) {
			((Measurements*)data)->reverseTemp = TRUE;      //reverse pattern
			if (ind % 2 == 0) {                                   //even tick
				((Measurements*)data)->tempRawBuff[j] -= 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->tempRawBuff[j] += 1;
			}
		}
		else {
			if (ind % 2 == 0) {                                   //even tick
				((Measurements*)data)->tempRawBuff[j] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->tempRawBuff[j] -= 1;
			}
		}
	}
	else {                                              //decreasing pattern
		if (((Measurements*)data)->tempRawBuff[j] < 15) {
			((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
			if (ind % 2 == 0) {                                   //even tick
				((Measurements*)data)->tempRawBuff[j] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->tempRawBuff[j] -= 1;
			}
		}
		else {
			if (ind % 2 == 0) {                                   //even tick
				((Measurements*)data)->tempRawBuff[j] -= 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->tempRawBuff[j] += 1;
			}
		}
	}
  }
	//systolic/diastolic pressure
	if (((Measurements*)data)->sysComplete == FALSE) {   //run systolic
		if (((Measurements*)data)->bloodPressRawBuff[j] > 100) {      //systolic complete
			((Measurements*)data)->sysComplete = TRUE;      //run diastolic
			((Measurements*)data)->bloodPressRawBuff[j] = 80;          //rest systolic
			if (ind % 2 == 0) {                                   //even tick
				((Measurements*)data)->bloodPressRawBuff[j+8] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->bloodPressRawBuff[j+8] -= 1;
			}
		}
		else {
			if (ind % 2 == 0) {                                   //even tick
				((Measurements*)data)->bloodPressRawBuff[j] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->bloodPressRawBuff[j] -= 1;
			}
		}
	}
	else {                                              //run diastolic
		if (((Measurements*)data)->bloodPressRawBuff[j] < 40) {       //diastolic complete
			((Measurements*)data)->sysComplete = FALSE;     //run systolic
			((Measurements*)data)->bloodPressRawBuff[j] = 80;          //reset diastolic
			if (ind % 2 == 0) {                                   //even tick
				((Measurements*)data)->bloodPressRawBuff[j] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->bloodPressRawBuff[j] -= 1;
			}
		}
		else {
			if (ind % 2 == 0) {                                   //even tick
				((Measurements*)data)->bloodPressRawBuff[j+8] += 2;
			}
			else {                                          //odd tick
				((Measurements*)data)->bloodPressRawBuff[j+8] -= 1;
			}
		}
	}

	//heartrate
	if (((Measurements*)data)->reversePulse == FALSE) {   //increasing pattern
		if (((Measurements*)data)->pulseRateRawBuff[j] > 40) {
			((Measurements*)data)->reversePulse = TRUE;     //reverse pattern
			if (pulseTime % 2 == 0) {                                   //even tick
				((Measurements*)data)->pulseRateRawBuff[j] += 1;
			}
			else {                                          //odd tick
				((Measurements*)data)->pulseRateRawBuff[j] -= 3;
			}
		}
		else {
			if (pulseTime % 2 == 0) {                                   //even tick
				((Measurements*)data)->pulseRateRawBuff[j] -= 1;
			}
			else {                                          //odd tick
				((Measurements*)data)->pulseRateRawBuff[j] += 3;
			}
		}
	}
	else {                                              //decreasing pattern
		if (((Measurements*)data)->pulseRateRawBuff[j] < 15) {
			((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
			if (pulseTime % 2 == 0) {                                   //even tick
				((Measurements*)data)->pulseRateRawBuff[j] -= 1;
			}
			else {                                          //odd tick
				((Measurements*)data)->pulseRateRawBuff[j] += 3;
			}
		}
		else {
			if (pulseTime % 2 == 0) {                                   //even tick
				((Measurements*)data)->pulseRateRawBuff[j] += 1;
			}
			else {                                          //odd tick
				((Measurements*)data)->pulseRateRawBuff[j] -= 3;
			}
		}
	}
        addFlags[1] = 1;
}

void compute(void* data) {
	float t = (float)(((ComputeData*)data)->tempRawBuff[j]);
	unsigned int s = (((ComputeData*)data)->bloodPressRawBuff[j]);
	unsigned int d = (((ComputeData*)data)->bloodPressRawBuff[j+8]); 
	unsigned int h = (((ComputeData*)data)->pulseRateRawBuff[j]);

	t = (5 + (0.75*t));
	s = 9 + (2 * s);
	d = (int)(6 + (1.5*d));
	h = 8 + (3 * h);
	((ComputeData*)data)->tempCorrectedBuff[j] = t;
	((ComputeData*)data)->bloodPressCorrectedBuff[j] = s;
	((ComputeData*)data)->bloodPressCorrectedBuff[j+8] = d;
	((ComputeData*)data)->pulseRateCorrectedBuff[j] = h;
        
        /*indexToDel = j;
        measureDelete = 1;
        if(j == 7){
          j = 0;
        }
        else {
          j++;
        }*/
        
        //addFlags[1] = 0;
        //addFlags[2] = 1;
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
      //addFlags[0] = 1;
      break;
    case 2:
      *m = 6;
      //addFlags[0] = 1;
      break;
    case 3:
      *m = 7;
      //addFlags[0] = 1;
      break;
    case 4:
      *m = 8;
      //addFlags[0] = 1;
      break;
    case 5:
      bpHigh = FALSE;
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
      intPrint(((Display*)data)->bloodPressCorrectedBuff[j], 3, 0, 2);         //Systolic: should never be over 3 char
      print("/", 3, 2);
      intPrint(((Display*)data)->bloodPressCorrectedBuff[j+8], 5, 4, 2);         //Diastolic: should never be over 5 char
      print("mm Hg", 9, 2);
    }
  
    if( *m == TEMP){  // temperature
      print("Temperature:", 0, 1);
      fPrint(((Display*)data)->tempCorrectedBuff[j], 4, 0, 2);               //Temperature: should never be over 4 char
      print("C", 4, 2);
    }
    
    if( *m == HR){  // heart rate
      print("Heart Rate:", 0, 1);
      intPrint(((Display*)data)->pulseRateCorrectedBuff[j], 3, 0, 2);        //Heartrate: should never be over 3 char
      print("BPM", 3, 2);
    }
  }
   
  if( *m == ANNUNCIATE){  // Annunciate Mode
    print("Annunciate:", 0, 0);
    volatile int t = *((Display*)data)->bloodPressCorrectedBuff;
    intPrint(((Display*)data)->bloodPressCorrectedBuff[j], 3, 0, 1);         //Systolic: should never be over 3 char
    print("/", 3, 1);
    intPrint(((Display*)data)->bloodPressCorrectedBuff[j+8], 5, 4, 1);         //Diastolic: should never be over 5 char
    print("mm Hg", 9, 1);

    fPrint(((Display*)data)->tempCorrectedBuff[j], 4, 0, 2);               //Temperature: should never be over 4 char
    print("C", 4, 2);
    intPrint(((Display*)data)->pulseRateCorrectedBuff[j], 3, 6, 2);        //Heartrate: should never be over 3 char
    print("BPM", 9, 2);

    intPrint(*((Display*)data)->batteryState, 3, 13, 2);    //battery: should never be over 3 char
  }

}

void annunciate(void* data) {
  
    float t = (float)((WarningAlarm*)data)->tempRawBuff[j];
    unsigned int s = ((WarningAlarm*)data)->bloodPressRawBuff[j];
    float d = (float)((WarningAlarm*)data)->bloodPressRawBuff[j+8];
    unsigned int h = ((WarningAlarm*)data)->pulseRateRawBuff[j];
    short b = *(((WarningAlarm*)data)->batteryState);

    t = 5 + (0.75*t);
    s = 9 + (2 * s);
    d = 6 + (1.5*d);
    h = 8 + (3 * h);

    if(b < 40 || h > 100 || h < 60 || t > 37.8 || t < 36.1 || 
       s > 120 || s < 90 || d > 80 || d < 60 ){
         
      if (b < 40 && time % 30 == 0) { // three seconds have passed
        if( GPIO_PORTF_DATA_R == ~(0x01)){ // LED is off
          GPIO_PORTF_DATA_R |= 0x01; // Turn on the LED
        }else{
          GPIO_PORTF_DATA_R &= ~(0x01); // Turn off the LED
        }
      }
  
      if (h > 100 || h < 60) {
        if(time % 20 == 0){ // two second have passed
         if( GPIO_PORTF_DATA_R == ~(0x01)){ // LED is off
          GPIO_PORTF_DATA_R |= 0x01; // Turn on the LED
          }else{
            GPIO_PORTF_DATA_R &= ~(0x01); // Turn off the LED
          }
        } 
      }
      
      if (t > 37.8 || t < 36.1) {
        if(time % 10 == 0){ // one second has passed
         if( GPIO_PORTF_DATA_R == ~(0x01)){ // LED is off
          GPIO_PORTF_DATA_R |= 0x01; // Turn on the LED
          }else{
            GPIO_PORTF_DATA_R &= ~(0x01); // Turn off the LED
          }
        } 
      }
  
      if (s > 120 || s < 90 || d > 80 || d < 60) {
        if(time % 5 == 0){ // one half-second has passed
         if( GPIO_PORTF_DATA_R == ~(0x01)){ // LED is off
          GPIO_PORTF_DATA_R |= 0x01; // Turn on the LED
          }else{
            GPIO_PORTF_DATA_R &= ~(0x01); // Turn off the LED
          }
        }
        bpHigh = TRUE;
      }else{
        bpHigh = FALSE;
      }
  
      if(s > 144 && bpHigh == TRUE){ // 20% over max systolic
        if(time % 10 == 0){ // one second has passed
         if( speakerOn == FALSE){ // speaker is off
          PWMGenEnable(PWM_BASE, PWM_GEN_0); // Turn on the speaker
          speakerOn = TRUE;
          }else{
            PWMGenDisable(PWM_BASE, PWM_GEN_0); // Turn off the speaker
            speakerOn = FALSE;
          }
        }
      }else{
        PWMGenDisable(PWM_BASE, PWM_GEN_0); // Turn off the speaker
      }
    
    }else{ // no warnings
      GPIO_PORTF_DATA_R |= 0x01; // Turn on the LED
    }
}


void status(void* data) {
	*(((Status*)data)->batteryState) -= 1;              //decrement battery by 1
}

void schedule(void* data) {
        /*if(measureDelete != 0){
          resetMeasureBufferAt(indexToDel);
          measureDelete = 0;
        }*/
        for (int j = 0; j < NUMTASKS; j++){
          if(addFlags[j] != 0){
            addFlags[j] = 0;
            llEnqueue(taskQueue,&tasks[j]);
          }
        }
        if(time%10 == 0 && (time/10)%2 == 0){
          addFlags[0] = 1; //flag measure
          addFlags[4] = 1; //flag status
        }
}

void delay(unsigned long aValue) {
    /*while(aValue--){
        while(SysTickValueGet() > 1000){
        }
        while(SysTickValueGet() < 1000){
        }
    }*/
   
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
    bloodPressRawBuff[i+8] = 80;
    pulseRateRawBuff[i] = 0;
  }
}

void resetMeasureBufferAt(int j) {
    tempRawBuff[j] = 75;
    bloodPressRawBuff[j] = 80;
    bloodPressRawBuff[j+8] = 80;
    pulseRateRawBuff[j] = 0;
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
                ll->tail = NULL;
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