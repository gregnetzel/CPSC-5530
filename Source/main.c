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

int main(void)
{

    //  define some local variables
    volatile int i = 0;
    unsigned long locPrintDelay = 10;
    unsigned long locDelDelay = 5;
    RIT128x96x4Init(1000000);
    //
    // The value if i is:
    //
    RIT128x96x4StringDraw("The value of i is:", 0, 0, 15);
    
    //
    //  print the digits 9 8 7 6 5 4 3 2 1 0
    while(TRUE)
    {
      for(i = 9; i >= 0; i--){
        print(i + '0',9-i, 1,&locPrintDelay);
      }
      for (i = 0; i < 10; i++){
        print(' ', i, 1,&locDelDelay);
      } 
    }
}
//Measure, Compute, Display, Annunciate, Status, Schedule
void measure(){
  //
}

void compute(){
  //
}

void display(){
  //
}

void annunciate(){
  //
}

void status(){
  //
}

void schedule(){
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
