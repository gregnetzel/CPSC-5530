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
#include "drivers/rit128x96x4.h"
#define TRUE 1
#define FALSE 0
#define hSpacing 8
#define vSpacing 14


struct MyStruct{
  void (*myTask)(void*);
  void* taskDataPtr;
};typedef struct MyStruct TCB;

struct Measurements{
  unsigned int temp;
  unsigned int sysPress;
  unsigned int diaPress;
  unsigned int heartRate;
  int reverseTemp;
  int sysComplete;
  int reversePulse;
};typedef struct Measurements Measurements;

struct Display{
  unsigned char* temp;
  unsigned char* sysPress;
  unsigned char* diaPress;
  unsigned char* heartRate;
};typedef struct Display Display;

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

void delay(unsigned long aValue);
void print(char* c, int hOffset, int vOffset);
void measure(void* data);
void compute();
void display();
void annunciate();
void status();
void schedule();
void fillStructs(Measurements* m, Display* d, Status* s, 
                 Warning* w, Alarms* a);

//*****************************************************************************
//
// DESCRIPTION
//
//*****************************************************************************

int main(void)
{
  //intitial data
    Measurements measurementData;
    Display displayData;
    Status statusData;
    Warning warningData;
    Alarms alarmData;
    
    fillStructs(&measurementData, &displayData, &statusData, 
                &warningData, &alarmData);
    
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
      taskManager[0].myTask(&measurementData);
      taskManager[1].myTask(NULL);
      taskManager[2].myTask(NULL); 
      taskManager[3].myTask(NULL);
      taskManager[4].myTask(NULL);
      taskManager[5].myTask(NULL);
      delay(100);
    }
}
//Measure, Compute, Display, Annunciate, Status, Schedule
void measure(void* data){
  print("MEASURE RUNNING", 0, 0);
  /* Access Data: ((type*)data)->member */
  
  //temperature 
  if(((Measurements*)data)->reverseTemp == FALSE){    //increasing pattern
    if( ((Measurements*)data)->temp > 50){
      ((Measurements*)data)->reverseTemp = TRUE;      //reverse pattern
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->temp -= 2;
      }else{                                          //odd tick
        ((Measurements*)data)->temp += 1;
      }*/
    }else{
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->temp += 2;
      }else{                                          //odd tick
        ((Measurements*)data)->temp -= 1;
      }*/
    }
  }else{                                              //decreasing pattern
    if( ((Measurements*)data)->temp < 15){
      ((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->temp += 2;
      }else{                                          //odd tick
        ((Measurements*)data)->temp -= 1;
      }*/
    }else{
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->temp -= 2;
      }else{                                          //odd tick
        ((Measurements*)data)->temp += 1;
      }*/
    }
  }

  //systolic/diatolic pressure NEED CONFIRMATION!!!
  if( ((Measurements*)data)->sysComplete == FALSE){   //run systolic
    if( ((Measurements*)data)->sysPress > 100){       //systolic complete
      ((Measurements*)data)->sysComplete = TRUE;      //run diatolic
      ((Measurements*)data)->sysPress = 80;           //rest systolic
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->diaPress += 2;
      }else{                                          //odd tick
        ((Measurements*)data)->diaPress -= 1;
      }*/
    }else{
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->sysPress += 2;
      }else{                                          //odd tick
        ((Measurements*)data)->sysPress -= 1;
      }*/
    }
  }else{                                              //run diatolic
    if( ((Measurements*)data)->sysPress < 40){        //diatolic complete
      ((Measurements*)data)->sysComplete = FALSE;     //run systolic
      ((Measurements*)data)->diaPress = 80;           //reset diatolic
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->sysPress += 2;
      }else{                                          //odd tick
        ((Measurements*)data)->sysPress -= 1;
      }*/
    }else{                                            
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->diaPress += 2;
      }else{                                          //odd tick
        ((Measurements*)data)->diaPress -= 1;
      }*/
    }
  }
  
  //heartrate
  if(((Measurements*)data)->reversePulse == FALSE){    //increasing pattern
    if( ((Measurements*)data)->heartRate > 40){
      ((Measurements*)data)->reversePulse = TRUE;      //reverse pattern
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->temp += 1;
      }else{                                          //odd tick
        ((Measurements*)data)->temp -= 3;
      }*/
    }else{
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->temp -= 1;
      }else{                                          //odd tick
        ((Measurements*)data)->temp += 3;
      }*/
    }
  }else{                                              //decreasing pattern
    if( ((Measurements*)data)->heartRate < 15){
      ((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->temp -= 1;
      }else{                                          //odd tick
        ((Measurements*)data)->temp += 3;
      }*/
    }else{
      /*if(i%2 == 0){                                   //even tick
        ((Measurements*)data)->temp += 1;
      }else{                                          //odd tick
        ((Measurements*)data)->temp -= 3;
      }*/
    }
  }
  
}

void compute(void* raw, void* calc){
  print("COMPUTE RUNNING", 0, 1);
  //((Measurements*)raw)->member ((Display*)calc)->member
  
  // may need type conversion???
  *((Display*)calc)->temp = (5 + (0.75*((Measurements*)raw)->temp));
  *((Display*)calc)->sysPress = (9 + (2*((Measurements*)raw)->sysPress));
  *((Display*)calc)->diaPress = (6 + (1.5*((Measurements*)raw)->diaPress));
  *((Display*)calc)->heartRate = (8 + (3*((Measurements*)raw)->diaPress));
}

void display(){
  print("DISPLAY RUNNING", 0, 2);
  //
}

void annunciate(){
  print("ANNUNCIATE RUNNING", 0, 3);
  //
}

void status(){
  print("STATUS RUNNING", 0, 4);
  //
}

void schedule(){
  print("SCHEDULE RUNNING", 0, 5);
  delay(10);
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

void fillStructs(Measurements* m, Display* d, Status* s, Warning* w, Alarms* a){
  //fill measurements
  m->temp = 75;
  m->sysPress = 80;
  m->diaPress = 80;
  m->heartRate = 50;
  m->reverseTemp = FALSE;
  m->sysComplete = FALSE;
  m->reversePulse = FALSE;

  //fill display
  d->temp = NULL;
  d->sysPress = NULL;
  d->diaPress = NULL;
  d->heartRate = NULL;
  
  //fill status
  s->batteryState = 200;
  
  //fill warning
  w->bpOOR = '0';
  w->pressOOR = '0';
  w->tempOOR = '0';
  
  //fill alarms
  a->bpHigh = FALSE;
  a->tempHigh = FALSE;
  a->pulseLow = FALSE;
}