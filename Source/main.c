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
#define TRUE 1
#define FALSE 0
#define hSpacing 8
#define vSpacing 12

void delay(unsigned long* aValue);
void print(char c, int hOffset, int vOffset, unsigned long* delay1);
void measure();
void compute();
void display();
void annunciate();
void status();
void schedule();
//*****************************************************************************
//
// Print "0123456789" 
//
//*****************************************************************************

struct MyStruct{
  void (*myTask)(void*);
  void* taskDataPtr;
};typedef struct MyStruct TCB;

struct Measurements{
  unsigned int temp;
  unsigned int sysPress;
  unsigned int diaPress;

  unsigned int heartRate;
};
typedef struct Measurements Measurements;

struct Display{
  unsigned char* temp;
  unsigned char* sysPress;
  unsigned char* diaPress;

  unsigned char* heartRate;
};
typedef struct Display Display;

struct Status{
  unsigned short batteryState;
};typedef struct Status Status;

struct Warning{
  unsigned char bpOOR;
  unsigned char pressOOR;
  unsigned char tempOOR;

};typedef struct Warning Warning;

struct Alarms{
  int bpHigh;
  int tempHigh;
  int pulseLow;

};typedef struct Alarms Alarms;

int main(void)
{
     //  define some local variables
    volatile int i = 0;
    RIT128x96x4Init(1000000);
    TCB taskManager[6];
    taskManager[0].myTask = measure;
    taskManager[1].myTask = compute;
    taskManager[2].myTask = display;
    taskManager[3].myTask = annunciate;
    taskManager[4].myTask = status;
    taskManager[5].myTask = schedule;
    while(TRUE)
    {
      
    }
}
//Measure, Compute, Display, Annunciate, Status, Schedule
void measure(){
  RIT128x96x4StringDraw("MEASURE RUNNING", 0, 0, 15);
}

void compute(){
  RIT128x96x4StringDraw("COMPUTE RUNNING", 0, 0, 15);
  //
}

void display(){
  RIT128x96x4StringDraw("DISPLAY RUNNING", 0, 0, 15);
  //
}

void annunciate(){
  RIT128x96x4StringDraw("ANNUNCIATE RUNNING", 0, 0, 15);
  //
}

void status(){
  RIT128x96x4StringDraw("STATUS RUNNING", 0, 0, 15);
  //
}

void schedule(){
  RIT128x96x4StringDraw("SCHEDULE RUNNING", 0, 0, 15);
  //
}

void delay(unsigned long* aValue){
    volatile unsigned long i = 0;
    volatile int j = 0;
    for (i = *aValue; i > 0; i--){
        for (j = 0; j < 100000; j++);
    }
    return;
}

void print(char c, int hOffset, int vOffset, unsigned long* delay1){
  char myData[3];
  myData[0] = c;              
  myData[1] = '\0';
  RIT128x96x4StringDraw(myData, hSpacing*(hOffset), vSpacing*(vOffset), 15);
  delay(delay1);
}
