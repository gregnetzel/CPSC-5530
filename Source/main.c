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
#define TRUE 1
#define FALSE 0
#define hSpacing 8
#define vSpacing 14

volatile int i = 0;
volatile unsigned long ulLoop;

//Data Variables
unsigned int tempRaw = 70;
unsigned int sysPressRaw = 80;
unsigned int diaPressRaw = 80;
unsigned int heartRateRaw = 50;

unsigned char* tempCorrected[5];
unsigned char* sysPressCorrected[4];
unsigned char* diaPressCorrected[6];
unsigned char* heartRateCorrected[4];

unsigned short batteryState = 200;

unsigned char bpOOR = '0';
unsigned char pressOOR = '0';
unsigned char tempOOR = '0';

int bpHigh = FALSE;
int tempHigh = FALSE;
int pulseLow = FALSE;

//Structs
struct MyStruct{
  void (*myTask)(void*);
  void* taskDataPtr;
};typedef struct MyStruct TCB;

struct Measurements{
  unsigned int* temp;
  unsigned int* sysPress;
  unsigned int* diaPress;
  unsigned int* heartRate;
  int reverseTemp;
  int sysComplete;
  int reversePulse;
};typedef struct Measurements Measurements;

struct ComputeData{
  unsigned int* tempRaw;
  unsigned int* sysPressRaw;
  unsigned int* diaPressRaw;
  unsigned int* heartRateRaw;
  unsigned char** tempCorrected;
  unsigned char** sysPressCorrected;
  unsigned char** diaPressCorrected;
  unsigned char** heartRateCorrected;
};typedef struct ComputeData ComputeData;

struct Display{
  unsigned char** temp;
  unsigned char** sysPress;
  unsigned char** diaPress;
  unsigned char** heartRate;
  unsigned short* batteryState;
};typedef struct Display Display;

struct Status{
  unsigned short* batteryState;
};typedef struct Status Status;

struct WarningAlarm{
  unsigned int* temp;
  unsigned int* sysPress;
  unsigned int* diaPress;
  unsigned int* heartRate;
  unsigned short* batteryState;
};typedef struct WarningAlarm WarningAlarm;

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
                 WarningAlarm* w);
void intToStr(int num, int size, unsigned char* str);

//*****************************************************************************
//
// DESCRIPTION
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

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;
  
    //
    // Do a dummy read to insert a few cycles after enabling the peripheral.
    //
    ulLoop = SYSCTL_RCGC2_R;
  
    //
    // Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIO_PORTF_DIR_R = 0x01;
    GPIO_PORTF_DEN_R = 0x01;
    
    fillStructs(&measurementData, &computeData, &displayData, &statusData, 
                &warningAlarmData);
    
    RIT128x96x4Init(1000000);
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
    while(TRUE)
    {
      taskManager[0].myTask(taskManager[0].taskDataPtr);
      taskManager[1].myTask(taskManager[1].taskDataPtr);
      taskManager[2].myTask(taskManager[2].taskDataPtr); 
      taskManager[3].myTask(taskManager[3].taskDataPtr);
      taskManager[4].myTask(taskManager[4].taskDataPtr);
      taskManager[5].myTask(taskManager[5].taskDataPtr);
      delay(100);
    }
}
//Measure, Compute, Display, Annunciate, Status, Schedule
void measure(void* data){
  //print("MEASURE RUNNING", 0, 3);
  /* Access Data: ((type*)data)->member */
  
  //temperature 
  if(((Measurements*)data)->reverseTemp == FALSE){    //increasing pattern
    if( *((Measurements*)data)->temp > 50){
      ((Measurements*)data)->reverseTemp = TRUE;      //reverse pattern
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->temp -= 2;
      }else{                                          //odd tick
        *((Measurements*)data)->temp += 1;
      }
    }else{
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->temp += 2;
      }else{                                          //odd tick
        *((Measurements*)data)->temp -= 1;
      }
    }
  }else{                                              //decreasing pattern
    if( *((Measurements*)data)->temp < 15){
      ((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->temp += 2;
      }else{                                          //odd tick
        *((Measurements*)data)->temp -= 1;
      }
    }else{
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->temp -= 2;
      }else{                                          //odd tick
        *((Measurements*)data)->temp += 1;
      }
    }
  }

  //systolic/diatolic pressure NEED CONFIRMATION!!!
  if( ((Measurements*)data)->sysComplete == FALSE){   //run systolic
    if( *((Measurements*)data)->sysPress > 100){       //systolic complete
      ((Measurements*)data)->sysComplete = TRUE;      //run diatolic
      *((Measurements*)data)->sysPress = 80;           //rest systolic
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->diaPress += 2;
      }else{                                          //odd tick
        *((Measurements*)data)->diaPress -= 1;
      }
    }else{
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->sysPress += 2;
      }else{                                          //odd tick
        *((Measurements*)data)->sysPress -= 1;
      }
    }
  }else{                                              //run diatolic
    if( *((Measurements*)data)->sysPress < 40){        //diatolic complete
      ((Measurements*)data)->sysComplete = FALSE;     //run systolic
      *((Measurements*)data)->diaPress = 80;           //reset diatolic
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->sysPress += 2;
      }else{                                          //odd tick
        *((Measurements*)data)->sysPress -= 1;
      }
    }else{                                            
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->diaPress += 2;
      }else{                                          //odd tick
        *((Measurements*)data)->diaPress -= 1;
      }
    }
  }
  
  //heartrate
  if(((Measurements*)data)->reversePulse == FALSE){    //increasing pattern
    if( *((Measurements*)data)->heartRate > 40){
      ((Measurements*)data)->reversePulse = TRUE;      //reverse pattern
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->temp += 1;
      }else{                                          //odd tick
        *((Measurements*)data)->temp -= 3;
      }
    }else{
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->temp -= 1;
      }else{                                          //odd tick
        *((Measurements*)data)->temp += 3;
      }
    }
  }else{                                              //decreasing pattern
    if( *((Measurements*)data)->heartRate < 15){
      ((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->temp -= 1;
      }else{                                          //odd tick
        *((Measurements*)data)->temp += 3;
      }
    }else{
      if(i%2 == 0){                                   //even tick
        *((Measurements*)data)->temp += 1;
      }else{                                          //odd tick
        *((Measurements*)data)->temp -= 3;
      }
    }
  }
  
}

void compute(void* data){
  //print("COMPUTE RUNNING", 0, 4);

  float t = (float)*(((ComputeData*)data)->tempRaw);
  unsigned int s = *(((ComputeData*)data)->sysPressRaw);
  float d = (float)*(((ComputeData*)data)->diaPressRaw);
  unsigned int h = *(((ComputeData*)data)->heartRateRaw);
  
  t = (int)(5 + (0.75*t));
  s = 9 + (2*s);
  d = (int)(6 + (1.5*d));
  h = 8 + (3*h);
  
  intToStr((int)t, 4, *((ComputeData*)data)->tempCorrected);
  intToStr(s, 3, *((ComputeData*)data)->sysPressCorrected);
  intToStr((int)d, 5, *((ComputeData*)data)->diaPressCorrected);
  intToStr(h, 3, *((ComputeData*)data)->heartRateCorrected);
}

void display(void* data){
  //print("DISPLAY RUNNING", 0, 5);
  //*((Display*)data)->member
  
  print((char*)*((Display*)data)->sysPress,0,0);                //Systolic: should never be over 3 char
  print("/",3,0);
  print((char*)*((Display*)data)->diaPress,4,0);              //Diastolic: should never be over 5 char (float)
  print("mm Hg",9,0);
  
  print((char*)*((Display*)data)->temp,0,1);               //Temperature: should never be over 4 char (float)
  print("C",3,1);
  print((char*)*((Display*)data)->heartRate,5,1);                //Heartrate: should never be over 3 char
  print("BPM",8,1);
  
  print((char*)*((Display*)data)->batteryState,13,1);               //battery: should never be over 3 char

}

void annunciate(void* data){
  //*((WarningAlarm*)data)->member

  float t = (float)*(((WarningAlarm*)data)->temp);
  unsigned int s = *(((WarningAlarm*)data)->sysPress);
  float d = (float)*(((WarningAlarm*)data)->diaPress);
  unsigned int h = *(((WarningAlarm*)data)->heartRate);
  short b = 0;//*(((WarningAlarm*)data)->batteryState); //causes a fault
  
  t = 5 + (0.75*t);
  s = 9 + (2*s);
  d = 6 + (1.5*d);
  h = 8 + (3*h);
  
  if(b < 40){
    //no instructions given
  }
  
  if(t > 37.8 || t < 36.1){
    //flash light at 1 second interval
    // Turn on the LED
    GPIO_PORTF_DATA_R |= 0x01;
    for(ulLoop = 0; ulLoop < 10000000; ulLoop++)
    {
    }
    GPIO_PORTF_DATA_R &= ~(0x01);               // Turn off the LED
    for(ulLoop = 0; ulLoop < 10000000; ulLoop++){}
    GPIO_PORTF_DATA_R |= 0x01;
    for(ulLoop = 0; ulLoop < 10000000; ulLoop++){}
    GPIO_PORTF_DATA_R &= ~(0x01);
    
  }
  
  if(h > 100 || h < 60){
    //flash light at 2 second interval
    // Turn on the LED
    GPIO_PORTF_DATA_R |= 0x01;
    for(ulLoop = 0; ulLoop < 2000000; ulLoop++)
    {
    }
    GPIO_PORTF_DATA_R &= ~(0x01);               // Turn off the LED
    for(ulLoop = 0; ulLoop < 20000000; ulLoop++){}
    GPIO_PORTF_DATA_R |= 0x01;
    for(ulLoop = 0; ulLoop < 20000000; ulLoop++){}
    GPIO_PORTF_DATA_R &= ~(0x01);
  }
  
  if(s > 120 || s < 90 || d > 80 || d < 60){
    //flash light at 0.5 second interval
    // Turn on the LED
    GPIO_PORTF_DATA_R |= 0x01;
    for(ulLoop = 0; ulLoop < 500000; ulLoop++)
    {
    }
    GPIO_PORTF_DATA_R &= ~(0x01);               // Turn off the LED
    for(ulLoop = 0; ulLoop < 500000; ulLoop++){}
    GPIO_PORTF_DATA_R |= 0x01;
    for(ulLoop = 0; ulLoop < 500000; ulLoop++){}
    GPIO_PORTF_DATA_R &= ~(0x01);
  }
  
  //
  // Turn on the LED.
  //
//  GPIO_PORTF_DATA_R |= 0x01;
//  
//  for(ulLoop = 0; ulLoop < 200000; ulLoop++)
//  {
//  }
//  
//  GPIO_PORTF_DATA_R &= ~(0x01);                 // Turn LED off
  
  //
  // Delay for a bit.
  //
//  for(ulLoop = 0; ulLoop < 200000; ulLoop++)
//  {
//  }
}


void status(void* data){
  //print("STATUS RUNNING", 0, 7);
  ((Status*)data)->batteryState --;              //decrement battery by 1
}

void schedule(void* data){
  //print("SCHEDULE RUNNING", 0, 8);
  delay(10);
  i++;
}

void delay(unsigned long aValue){
    volatile unsigned long i = 0;
    volatile int j = 0;
    for (i = aValue; i > 0; i--){
        for (j = 0; j < 100000; j++);
    }
    return;
}

void print(char* c, int hOffset, int vOffset){// string, column, row
  RIT128x96x4StringDraw(c, hSpacing*(hOffset), vSpacing*(vOffset), 15);
}

void fillStructs(Measurements* m, ComputeData* c, Display* d, Status* s, WarningAlarm* w){
  //measure
  m->temp = &tempRaw;
  m->sysPress = &sysPressRaw;
  m->diaPress = &diaPressRaw;
  m->heartRate = &heartRateRaw;
  m->reverseTemp = FALSE;
  m->sysComplete = FALSE;
  m->reversePulse = FALSE;
  
  //compute
  c->tempRaw = &tempRaw;
  c->sysPressRaw = &sysPressRaw;
  c->diaPressRaw = &diaPressRaw;
  c->heartRateRaw = &heartRateRaw;
  c->tempCorrected = tempCorrected;
  c->sysPressCorrected = sysPressCorrected;
  c->diaPressCorrected = diaPressCorrected;
  c->heartRateCorrected = heartRateCorrected;
  
  //display
  d->temp = tempCorrected;
  d->sysPress = sysPressCorrected;
  d->diaPress = diaPressCorrected;
  d->heartRate = heartRateCorrected;
  d->batteryState = &batteryState;
  
  //status
  s->batteryState = &batteryState;
  
  //warningAlarm
  w->temp = &tempRaw;
  w->sysPress = &sysPressRaw;
  w->diaPress = &diaPressRaw;
  w->heartRate = &heartRateRaw;
}

void intToStr(int num, int size, unsigned char* str){//number, size, and memory location
  str[size+1] = '\0';
  volatile int c;
  volatile int j;
  for(j = 0; j < size; j++){
    str[size-j] = num%10+'0';
    num /= 10;
  }
}