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
#include "drivers/rit128x96x4.h"
#define TRUE 1                               //used for display to OLED
#define FALSE 0
#define hSpacing 8
#define vSpacing 14

volatile int i = 0;
volatile unsigned long ulLoop;

//Data Variables
unsigned int tempRawBuff[8];
unsigned int bloodPressRawBuff[16];			//sys in first half, dia in second (or something else).
unsigned int pulseRateRawBuff[8];

float tempCorrectedBuff[8];
unsigned int bloodPressCorrectedBuff[16];	//same comment as line 45
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
struct MyStruct {
	void(*myTask)(void*);
	void* taskDataPtr;
	struct MyStruct* next;
	struct MyStruct* prev;
}; typedef struct MyStruct TCB;

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
	unsigned int* bloodPressCorrectedBuf;
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
}; typedef struct Communications Communications;

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
void intPrint(int c, int size, int hOffset, int vOffset);
void fPrint(float c, int size, int hOffset, int vOffset);
void llEnqueue(LinkedList* ll, TCB* task);
TCB* llDequeue(LinkedList* ll);
//*****************************************************************************
//
// Simulated Medical Device
//
//*****************************************************************************

int main(void)
{
	//intitial data
	Measurements measurementData;
	ComputeData computeData;
	Display displayData;
	Status statusData;
	WarningAlarm warningAlarmData;
	Keypad keypadData;
	Communications communicationsData;

	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;  // Enable the GPIO port that is used for the on-board LED.

	ulLoop = SYSCTL_RCGC2_R;              // Do a dummy read to insert a few cycles after enabling the peripheral.

	GPIO_PORTF_DIR_R = 0x01;              // Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
	GPIO_PORTF_DEN_R = 0x01;              // enable the GPIO pin for digital function.

	fillStructs(&measurementData, &computeData, &displayData, &statusData,
		&warningAlarmData, &keypadData, &communicationsData);

	RIT128x96x4Init(1000000); //crashes here for some reason
	TCB taskManager[6];
	taskManager[0].myTask = measure;
	taskManager[1].myTask = compute;
	taskManager[2].myTask = display;
	taskManager[3].myTask = annunciate;
	taskManager[4].myTask = status;
	taskManager[5].myTask = schedule;

	taskManager[0].taskDataPtr = &measurementData;
	taskManager[1].taskDataPtr = &computeData;
	taskManager[2].taskDataPtr = &displayData;
	taskManager[3].taskDataPtr = &warningAlarmData;
	taskManager[4].taskDataPtr = &statusData;
	taskManager[5].taskDataPtr = NULL;
	while (TRUE)
	{
		/*taskManager[0].myTask(taskManager[0].taskDataPtr);
		taskManager[1].myTask(taskManager[1].taskDataPtr);
		taskManager[2].myTask(taskManager[2].taskDataPtr);
		taskManager[3].myTask(taskManager[3].taskDataPtr);
		taskManager[4].myTask(taskManager[4].taskDataPtr);
		taskManager[5].myTask(taskManager[5].taskDataPtr);
		*/
	}
}

void measure(void* data) {/*
	//temperature
	if (((Measurements*)data)->reverseTemp == FALSE) {    //increasing pattern
		if (*((Measurements*)data)->temp > 50) {
			((Measurements*)data)->reverseTemp = TRUE;      //reverse pattern
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->temp -= 2;
			}
			else {                                          //odd tick
				*((Measurements*)data)->temp += 1;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->temp += 2;
			}
			else {                                          //odd tick
				*((Measurements*)data)->temp -= 1;
			}
		}
	}
	else {                                              //decreasing pattern
		if (*((Measurements*)data)->temp < 15) {
			((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->temp += 2;
			}
			else {                                          //odd tick
				*((Measurements*)data)->temp -= 1;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->temp -= 2;
			}
			else {                                          //odd tick
				*((Measurements*)data)->temp += 1;
			}
		}
	}
	//systolic/diastolic pressure
	if (((Measurements*)data)->sysComplete == FALSE) {   //run systolic
		if (*((Measurements*)data)->sysPress > 100) {      //systolic complete
			((Measurements*)data)->sysComplete = TRUE;      //run diatolic
			*((Measurements*)data)->sysPress = 80;          //rest systolic
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->diaPress += 2;
			}
			else {                                          //odd tick
				*((Measurements*)data)->diaPress -= 1;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->sysPress += 2;
			}
			else {                                          //odd tick
				*((Measurements*)data)->sysPress -= 1;
			}
		}
	}
	else {                                              //run diatolic
		if (*((Measurements*)data)->sysPress < 40) {       //diatolic complete
			((Measurements*)data)->sysComplete = FALSE;     //run systolic
			*((Measurements*)data)->diaPress = 80;          //reset diatolic
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->sysPress += 2;
			}
			else {                                          //odd tick
				*((Measurements*)data)->sysPress -= 1;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->diaPress += 2;
			}
			else {                                          //odd tick
				*((Measurements*)data)->diaPress -= 1;
			}
		}
	}

	//heartrate
	if (((Measurements*)data)->reversePulse == FALSE) {   //increasing pattern
		if (*((Measurements*)data)->heartRate > 40) {
			((Measurements*)data)->reversePulse = TRUE;     //reverse pattern
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->temp += 1;
			}
			else {                                          //odd tick
				*((Measurements*)data)->temp -= 3;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->temp -= 1;
			}
			else {                                          //odd tick
				*((Measurements*)data)->temp += 3;
			}
		}
	}
	else {                                              //decreasing pattern
		if (*((Measurements*)data)->heartRate < 15) {
			((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->temp -= 1;
			}
			else {                                          //odd tick
				*((Measurements*)data)->temp += 3;
			}
		}
		else {
			if (i % 2 == 0) {                                   //even tick
				*((Measurements*)data)->temp += 1;
			}
			else {                                          //odd tick
				*((Measurements*)data)->temp -= 3;
			}
		}
	}
*/
}

void compute(void* data) {
/*
	float t = (float)*(((ComputeData*)data)->tempRaw);
	unsigned int s = *(((ComputeData*)data)->sysPressRaw);
	float d = (float)*(((ComputeData*)data)->diaPressRaw);
	unsigned int h = *(((ComputeData*)data)->heartRateRaw);

	t = (5 + (0.75*t));
	s = 9 + (2 * s);
	d = (int)(6 + (1.5*d));
	h = 8 + (3 * h);
	*((ComputeData*)data)->tempCorrected = t;
	*((ComputeData*)data)->sysPressCorrected = s;
	*((ComputeData*)data)->diaPressCorrected = (int)d;
	*((ComputeData*)data)->heartRateCorrected = h;
  */
}

void display(void* data) {
  /*
  TODO:
    add field to data for Menu Mode vs Annunciate Mode vs menu options
    print statements for menu options
    print statements for heart rate, blood pressure, & temp
  */
  
  /*  
  if( ? ){  // Menu Mode
    print("Menu", 0, 0);
  
    //  Blood Pressure hover
    if( ? ){
      print("* Blood Pressure        ", 0, 1);
    }else{
      print("  Blood Pressure        ", 0, 1);
    }
  
    print("              ", 0, 2);  // clear row
  
    //  Temperature hover
    if( ? ){
      print("* Temperature", 0, 3);
    }else{
      print("  Temperature", 0, 3);
    }
  
    //  Heart Rate hover
    if( ? ){
      print("* Heart Rate", 0, 5);
    }else{
      print("  Heart Rate", 0, 5);
    }
  }
  */
  
  /*  Heart Rate Mode
  
  */
  
  /*  Blood Pressure Mode
  
  */
  
  /*  Temp Mode
  
  */

  /*    Annunciate Mode
	volatile int t = *((Display*)data)->sysPress;
	intPrint(*((Display*)data)->sysPress, 3, 0, 0);         //Systolic: should never be over 3 char
	print("/", 3, 0);
	intPrint(*((Display*)data)->diaPress, 5, 4, 0);         //Diastolic: should never be over 5 char
	print("mm Hg", 9, 0);

	fPrint(*((Display*)data)->temp, 4, 0, 1);               //Temperature: should never be over 4 char
	print("C", 4, 1);
	intPrint(*((Display*)data)->heartRate, 3, 6, 1);        //Heartrate: should never be over 3 char
	print("BPM", 9, 1);

	intPrint(*((Display*)data)->batteryState, 3, 13, 1);    //battery: should never be over 3 char
  */

}

void annunciate(void* data) {
/*
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

	delay(10);
	i++;
}

void delay(unsigned long aValue) {
	volatile unsigned long i = 0;
	volatile int j = 0;
	for (i = aValue; i > 0; i--) {
		for (j = 0; j < 100000; j++);
	}
	return;
}

void print(char* c, int hOffset, int vOffset) {                        // string, column, row
	RIT128x96x4StringDraw(c, hSpacing*(hOffset), vSpacing*(vOffset), 15);
}

void intPrint(int c, int size, int hOffset, int vOffset) {             // string, column, row
	char dec[2];
	dec[1] = '\0';
	int rem = c;
	for (int i = size - 1; i >= 0; i--) {
		dec[0] = rem % 10 + '0';
		rem = rem / 10;
		RIT128x96x4StringDraw(dec, hSpacing*(hOffset + i), vSpacing*(vOffset), 15);
	}
}
void fPrint(float c, int size, int hOffset, int vOffset) {
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

	c->tempRawBuff = &tempRawBuff[0];                              //compute
	c->bloodPressRawBuff = &bloodPressRawBuff[0];
	c->pulseRateRawBuff = &pulseRateRawBuff[0];
	c->tempCorrectedBuff = &tempCorrectedBuff[0];
	c->bloodPressCorrectedBuff = &bloodPressCorrectedBuff[0];
	c->pulseRateCorrectedBuff = &pulseRateCorrectedBuff[0];

	d->tempCorrectedBuff = &tempCorrectedBuff[0];                           //display
	d->bloodPressCorrectedBuf = &bloodPressCorrectedBuff[0];
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