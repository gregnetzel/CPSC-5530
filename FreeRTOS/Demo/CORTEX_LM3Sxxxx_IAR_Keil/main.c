/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt timing.
 * The maximum measured jitter time is latched in the ulMaxJitter variable, and
 * displayed on the OLED display by the 'OLED' task as described below.  The
 * fast interrupt is configured and handled in the timertest.c source file.
 *
 * "OLED" task - the OLED task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the display directly.  Other tasks wishing to write a
 * message to the OLED send the message on a queue to the OLED task instead of
 * accessing the OLED themselves.  The OLED task just blocks on the queue waiting
 * for messages - waking and displaying the messages as they arrive.
 *
 * "Check" hook -  This only executes every five seconds from the tick hook.
 * Its main function is to check that all the standard demo tasks are still
 * operational.  Should any unexpected behaviour within a demo task be discovered
 * the tick hook will write an error to the OLED (via the OLED task).  If all the
 * demo tasks are executing with their expected behaviour then the check task
 * writes PASS to the OLED (again via the OLED task), as described above.
 *
 * "uIP" task -  This is the task that handles the uIP stack.  All TCP/IP
 * processing is performed in this task.
 */




/*************************************************************************
 * Please ensure to read http://www.freertos.org/portlm3sx965.html
 * which provides information on configuring and running this demo for the
 * various Luminary Micro EKs.
 *************************************************************************/

/* Set the following option to 1 to include the WEB server in the build.  By
default the WEB server is excluded to keep the compiled code size under the 32K
limit imposed by the KickStart version of the IAR compiler.  The graphics
libraries take up a lot of ROM space, hence including the graphics libraries
and the TCP/IP stack together cannot be accommodated with the 32K size limit. */
#define mainINCLUDE_WEB_SERVER	0


/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware library includes. */
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_sysctl.h"
#include "sysctl.h"
#include "gpio.h"
#include "grlib.h"
#include "rit128x96x4.h"
#include "osram128x64x4.h"
#include "formike128x128x16.h"

/* Demo app includes. */
#include "BlockQ.h"
#include "death.h"
#include "integer.h"
#include "blocktim.h"
#include "flash.h"
#include "partest.h"
#include "semtest.h"
#include "PollQ.h"
#include "lcd_message.h"
#include "bitmap.h"
#include "GenQTest.h"
#include "QPeek.h"
#include "recmutex.h"
#include "IntQueue.h"
#include "QueueSet.h"
#include "EventGroupsDemo.h"

/* Health app includes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*-----------------------------------------------------------*/

/* The time between cycles of the 'check' functionality (defined within the
tick hook. */
#define mainCHECK_DELAY		( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* Size of the stack allocated to the uIP task. */
#define mainBASIC_WEB_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 3 )

/* The OLED task uses the sprintf function so requires a little more stack too. */
#define mainOLED_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCREATOR_TASK_PRIORITY       ( tskIDLE_PRIORITY + 3 )
#define mainINTEGER_TASK_PRIORITY       ( tskIDLE_PRIORITY )
#define mainGEN_QUEUE_TASK_PRIORITY     ( tskIDLE_PRIORITY )

/* The maximum number of message that can be waiting for display at any one
time. */
#define mainOLED_QUEUE_SIZE					( 3 )

/* Dimensions the buffer into which the jitter time is written. */
#define mainMAX_MSG_LEN						25

/* The period of the system clock in nano seconds.  This is used to calculate
the jitter time in nano seconds. */
#define mainNS_PER_CLOCK	( ( unsigned long ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Constants used when writing strings to the display. */
#define mainCHARACTER_HEIGHT			( 9 )
#define mainMAX_ROWS_128			( mainCHARACTER_HEIGHT * 14 )
#define mainMAX_ROWS_96				( mainCHARACTER_HEIGHT * 10 )
#define mainMAX_ROWS_64				( mainCHARACTER_HEIGHT * 7 )
#define mainFULL_SCALE				( 15 )
#define ulSSI_FREQUENCY				( 3500000UL )

/*-----------------------------------------------------------*/

/*
 * The task that handles the uIP stack.  All TCP/IP processing is performed in
 * this task.
 */
extern void vuIP_Task( void *pvParameters );

/*
 * The display is written two by more than one task so is controlled by a
 * 'gatekeeper' task.  This is the only task that is actually permitted to
 * access the display directly.  Other tasks wanting to display a message send
 * the message to the gatekeeper.
 */
static void vOLEDTask( void *pvParameters );

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/*
 * Configures the high frequency timers - those used to measure the timing
 * jitter while the real time kernel is executing.
 */
extern void vSetupHighFrequencyTimer( void );

/*
 * Hook functions that can get called by the kernel.
 */
void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName );
void vApplicationTickHook( void );


/*-----------------------------------------------------------*/

/* The queue used to send messages to the OLED task. */
QueueHandle_t xOLEDQueue;

/* The welcome text. */
const char * const pcWelcomeMessage = "   www.FreeRTOS.org";

/*-----------------------------------------------------------*/

/* Our Headers and stuff. */
#define NUMTASKS 8
volatile int i = 0;
volatile int j = 0;               // circular buffer usage
volatile int indexToDel = 0;
volatile unsigned long ulLoop;
volatile unsigned long time = 0; //in tenths of seconds
volatile unsigned long pulseTime = 0; //pulse transducer time

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
#define TRUE 1
#define FALSE 0
int bpHigh = FALSE;
int speakerOn = FALSE;
int pulseLow = FALSE;

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

//functions
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
void serialCommunications(void* data);
void startup();

/*************************************************************************
 * Please ensure to read http://www.freertos.org/portlm3sx965.html
 * which provides information on configuring and running this demo for the
 * various Luminary Micro EKs.
 *************************************************************************/

int main( void ){
  prvSetupHardware();
  
  /* Create the queue used by the OLED task.  Messages for display on the OLED
	are received via this queue. */
  xOLEDQueue = xQueueCreate( mainOLED_QUEUE_SIZE, sizeof( xOLEDMessage ) );
  
  /* Start the standard demo tasks. */
  vStartIntegerMathTasks( mainINTEGER_TASK_PRIORITY );
  vStartGenericQueueTasks( mainGEN_QUEUE_TASK_PRIORITY );
  vStartInterruptQueueTasks();
  vStartRecursiveMutexTasks();
  vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
  vCreateBlockTimeTasks();
  vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
  vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
  vStartQueuePeekTasks();
  vStartQueueSetTasks();
  vStartEventGroupTasks();
  
  /* Exclude some tasks if using the kickstart version to ensure we stay within
	the 32K code size limit. */
#if mainINCLUDE_WEB_SERVER != 0 
  {
    /* Create the uIP task if running on a processor that includes a MAC and
		PHY. */
    if( SysCtlPeripheralPresent( SYSCTL_PERIPH_ETH ) ) {
      xTaskCreate( vuIP_Task, "uIP", mainBASIC_WEB_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
    }
  }
#endif
  
  
  
  /* Start the tasks defined within this file/specific to this demo. */
  xTaskCreate( vOLEDTask, "OLED", mainOLED_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
  
  /* The suicide tasks must be created last as they need to know how many
	tasks were running prior to their creation in order to ascertain whether
	or not the correct/expected number of tasks are running at any given time. */
  vCreateSuicidalTasks( mainCREATOR_TASK_PRIORITY );
  
  /* Configure the high frequency interrupt used to measure the interrupt
	jitter time. */
  vSetupHighFrequencyTimer();
  
  
  /* Start the scheduler. */
  vTaskStartScheduler();
  
  /* Will only get here if there was insufficient memory to create the idle
    task. */
  return 0;
}
/*-----------------------------------------------------------*/

/* Our Functions */
//void startup(){
//  RIT128x96x4Init(1000000);             //light enable
//  SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;  // Enable the GPIO port that is used for the on-board LED.
//  ulLoop = SYSCTL_RCGC2_R;              // Do a dummy read to insert a few cycles after enabling the peripheral.
//  GPIO_PORTF_DIR_R = 0x01;              // Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
//  GPIO_PORTF_DEN_R = 0x01;              // enable the GPIO pin for digital function.
//  
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);  //uart enable for serial comm
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//  GPIOPinConfigure(GPIO_PA0_U0RX);
//  GPIOPinConfigure(GPIO_PA1_U0TX);
//  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1);
//  IntEnable(INT_UART0);
//  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
//  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8|UART_CONFIG_PAR_NONE|UART_CONFIG_STOP_ONE);
//  
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);          //up down buttons enabled
//  GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
//  GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_STRENGTH_2MA,
//                   GPIO_PIN_TYPE_STD_WPU);
//  GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_FALLING_EDGE);
//  GPIOPinIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
//  IntEnable(INT_GPIOE);
//  
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  //select button (for some reason on same port as light which is annoying)
//  GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
//  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA,
//                   GPIO_PIN_TYPE_STD_WPU);
//  GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
//  GPIOPinIntEnable(GPIO_PORTF_BASE, GPIO_PIN_1);
//  IntEnable(INT_GPIOF);
//  
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);    // speaker
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
//  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
//  GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
//  unsigned long ulPeriod = SysCtlClockGet() / 440;
//  PWMGenConfigure(PWM_BASE, PWM_GEN_0,
//                  PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
//  PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod);
//  PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, ulPeriod / 4);
//  PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ulPeriod * 3 / 4);
//  
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // global timer
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // pulse timer
//  IntMasterEnable();
//  TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
//  TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
//  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/10); // need to play with how often timer ISR is run I think right now its 100ms.
//  TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()*2);  // pulse transducer increments once every 2 seconds.
//  IntEnable(INT_TIMER0A);
//  IntEnable(INT_TIMER1A);
//  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
//  TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
//  TimerEnable(TIMER0_BASE, TIMER_A);
//  TimerEnable(TIMER1_BASE, TIMER_A);
//}
//
//void serialCommunications(void* data){
//  /*Communications* d = data;
//  sprintf(comBuffer,"1.Temperature:  %d\n2. Systolic pressure: %dmmHG\n3. Dyastolic pressure: %dmmHG\n4. Pulse rate: %dBPM\n5. Battery: %d",
//          d->tempCorrectedBuff[0],d->bloodPressCorrectedBuff[0],d->bloodPressCorrectedBuff[8],pulseRateCorrectedBuff[0],d->batteryState);
//  
//  char* buf = &comBuffer[0];
//  while(UARTBusy(UART0_BASE));
//  while(*buf != '\0'){
//    UARTCharPut(UART0_BASE, *buf++);
//  }
//  */
//}
//
//void measure(void* data) {
//  int ind = i/10;
//  if(i%10 == 0){                                          //temperature
//    if (((Measurements*)data)->reverseTemp == FALSE) {    //increasing pattern
//      if (((Measurements*)data)->tempRawBuff[j] > 50) {
//        ((Measurements*)data)->reverseTemp = TRUE;      //reverse pattern
//        if (ind % 2 == 0) {                                   //even tick
//          ((Measurements*)data)->tempRawBuff[j] -= 2;
//        }
//        else {                                          //odd tick
//          ((Measurements*)data)->tempRawBuff[j] += 1;
//        }
//      }
//      else {
//        if (ind % 2 == 0) {                                   //even tick
//          ((Measurements*)data)->tempRawBuff[j] += 2;
//        }
//        else {                                          //odd tick
//          ((Measurements*)data)->tempRawBuff[j] -= 1;
//        }
//      }
//    }
//    else {                                              //decreasing pattern
//      if (((Measurements*)data)->tempRawBuff[j] < 15) {
//        ((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
//        if (ind % 2 == 0) {                                   //even tick
//          ((Measurements*)data)->tempRawBuff[j] += 2;
//        }
//        else {                                          //odd tick
//          ((Measurements*)data)->tempRawBuff[j] -= 1;
//        }
//      }
//      else {
//        if (ind % 2 == 0) {                                   //even tick
//          ((Measurements*)data)->tempRawBuff[j] -= 2;
//        }
//        else {                                          //odd tick
//          ((Measurements*)data)->tempRawBuff[j] += 1;
//        }
//      }
//    }
//  }
//  //systolic/diastolic pressure
//  if (((Measurements*)data)->sysComplete == FALSE) {   //run systolic
//    if (((Measurements*)data)->bloodPressRawBuff[j] > 100) {      //systolic complete
//      ((Measurements*)data)->sysComplete = TRUE;      //run diastolic
//      ((Measurements*)data)->bloodPressRawBuff[j] = 80;          //rest systolic
//      if (ind % 2 == 0) {                                   //even tick
//        ((Measurements*)data)->bloodPressRawBuff[j+8] += 2;
//      }
//      else {                                          //odd tick
//        ((Measurements*)data)->bloodPressRawBuff[j+8] -= 1;
//      }
//    }
//    else {
//      if (ind % 2 == 0) {                                   //even tick
//        ((Measurements*)data)->bloodPressRawBuff[j] += 2;
//      }
//      else {                                          //odd tick
//        ((Measurements*)data)->bloodPressRawBuff[j] -= 1;
//      }
//    }
//  }
//  else {                                              //run diastolic
//    if (((Measurements*)data)->bloodPressRawBuff[j] < 40) {       //diastolic complete
//      ((Measurements*)data)->sysComplete = FALSE;     //run systolic
//      ((Measurements*)data)->bloodPressRawBuff[j] = 80;          //reset diastolic
//      if (ind % 2 == 0) {                                   //even tick
//        ((Measurements*)data)->bloodPressRawBuff[j] += 2;
//      }
//      else {                                          //odd tick
//        ((Measurements*)data)->bloodPressRawBuff[j] -= 1;
//      }
//    }
//    else {
//      if (ind % 2 == 0) {                                   //even tick
//        ((Measurements*)data)->bloodPressRawBuff[j+8] += 2;
//      }
//      else {                                          //odd tick
//        ((Measurements*)data)->bloodPressRawBuff[j+8] -= 1;
//      }
//    }
//  }
//  
//  //heartrate
//  if (((Measurements*)data)->reversePulse == FALSE) {   //increasing pattern
//    if (((Measurements*)data)->pulseRateRawBuff[j] > 40) {
//      ((Measurements*)data)->reversePulse = TRUE;     //reverse pattern
//      if (pulseTime % 2 == 0) {                                   //even tick
//        ((Measurements*)data)->pulseRateRawBuff[j] += 1;
//      }
//      else {                                          //odd tick
//        ((Measurements*)data)->pulseRateRawBuff[j] -= 3;
//      }
//    }
//    else {
//      if (pulseTime % 2 == 0) {                                   //even tick
//        ((Measurements*)data)->pulseRateRawBuff[j] -= 1;
//      }
//      else {                                          //odd tick
//        ((Measurements*)data)->pulseRateRawBuff[j] += 3;
//      }
//    }
//  }
//  else {                                              //decreasing pattern
//    if (((Measurements*)data)->pulseRateRawBuff[j] < 15) {
//      ((Measurements*)data)->reverseTemp = FALSE;     //reverse pattern
//      if (pulseTime % 2 == 0) {                                   //even tick
//        ((Measurements*)data)->pulseRateRawBuff[j] -= 1;
//      }
//      else {                                          //odd tick
//        ((Measurements*)data)->pulseRateRawBuff[j] += 3;
//      }
//    }
//    else {   
//      if (pulseTime % 2 == 0) {                                   //even tick
//        ((Measurements*)data)->pulseRateRawBuff[j] += 1;
//      }
//      else {                                          //odd tick
//        ((Measurements*)data)->pulseRateRawBuff[j] -= 3;
//      }
//    }
//  }
//  addFlags[1] = 1;
//}
//
//void compute(void* data) {
//  float t = (float)(((ComputeData*)data)->tempRawBuff[j]);
//  unsigned int s = (((ComputeData*)data)->bloodPressRawBuff[j]);
//  unsigned int d = (((ComputeData*)data)->bloodPressRawBuff[j+8]); 
//  unsigned int h = (((ComputeData*)data)->pulseRateRawBuff[j]);
//  
//  t = (5 + (0.75*t));
//  s = 9 + (2 * s);
//  d = (int)(6 + (1.5*d));
//  h = 8 + (3 * h);
//  ((ComputeData*)data)->tempCorrectedBuff[j] = t;
//  ((ComputeData*)data)->bloodPressCorrectedBuff[j] = s;
//  ((ComputeData*)data)->bloodPressCorrectedBuff[j+8] = d;
//  ((ComputeData*)data)->pulseRateCorrectedBuff[j] = h;
//}
//
///*MENU_HOVER = 0, ANNUN_HOVER = 1,
//HR_HOVER = 2, BP_HOVER = 3, 
//TEMP_HOVER = 4, ANNUNCIATE = 5, 
//HR = 6, BP = 7, TEMP = 8*/
//
//void display(void* data) {
//  /*unsigned short* m = ((Display*)data)->mode;
//  if (selectPressed == 1){
//    selectPressed = 0;
//    upPressed = 0;
//    downPressed = 0;
//    RIT128x96x4Clear();
//    
//    switch(*m) {
//    case 0:
//      *m = 2;
//      break;
//    case 1:
//      *m = 5;      
//      //addFlags[0] = 1;
//      break;
//    case 2:
//      *m = 6;
//      //addFlags[0] = 1;
//      break;
//    case 3:
//      *m = 7;
//      //addFlags[0] = 1;
//      break;
//    case 4:
//      *m = 8;
//      //addFlags[0] = 1;
//      break;
//    case 5:
//      bpHigh = FALSE;
//      break;
//    }
//  }
//  else{
//    if (upPressed == 1){
//      RIT128x96x4Clear();
//      switch(*m) {
//      case 0:
//        break;
//      case 1:
//        *m = 0;
//        break;
//      case 2:
//        break;
//      case 3:
//        *m = 2;
//        break;
//      case 4:
//        *m = 3;
//        break;
//      }
//      upPressed = 0;
//    }
//    if (downPressed == 1){
//      RIT128x96x4Clear();
//      switch(*m) {
//      case 0:
//        *m = 1;
//        break;
//      case 1:
//        break;
//      case 2:
//        *m = 3;
//        break;
//      case 3:
//        *m = 4;
//        break;
//      case 4:
//        break;
//      }
//      downPressed = 0;
//    }
//    if (leftPressed == 1){
//      RIT128x96x4Clear();
//      switch(*m) {
//      case 2:
//      case 3:
//      case 4:
//        *m = 0;
//        break;
//      case 5:
//        *m = 1;
//        break;
//      case 6:
//        *m = 2;
//        break;
//      case 7:
//        *m = 3;
//        break;
//      case 8:
//        *m = 4;
//        break;
//      }
//      leftPressed = 0;
//    }
//  }
//  if( *m == MENU_HOVER || *m == ANNUN_HOVER ){  // mode selection
//    print("Please select a Mode:", 0, 0);
//  
//    if( *m == MENU_HOVER){//  menu mode hover
//      print("* Menu", 0, 1);
//    }
//    else{
//      print("  Menu", 0, 1);
//    }
//  
//    if( *m == ANNUN_HOVER){//  annunciate hover
//      print("* Annunciate", 0, 2);
//    }
//    else{
//      print("  Annunciate", 0, 2);
//    }
//  }
//   
//  if( *m == BP_HOVER || *m == TEMP_HOVER || *m == HR_HOVER){  // Menu Mode
//    print("Menu", 0, 0);
//  
//    if( *m == HR_HOVER ){//  Heart Rate hover
//      print("* Heart Rate", 0, 1);
//    }
//    else{
//      print("  Heart Rate", 0, 1);
//    }
//  
//    if( *m == BP_HOVER){    //  Blood Pressure hover
//      print("* Blood Pressure", 0, 3);
//    }
//    else{
//      print("  Blood Pressure", 0, 3);
//    }
//  
//    if( *m == TEMP_HOVER ){    //  Temperature hover
//      print("* Temperature", 0, 5);
//    }
//    else{
//      print("  Temperature", 0, 5);
//    }
//  }
//  
//  if( *m == BP || *m == HR || *m == TEMP){  // Menu option display
//    print("Menu", 0, 0);
//    
//    if( *m == BP){  // Blood Pressure
//      print("Blood Pressure", 0, 1);
//      volatile int t = *((Display*)data)->bloodPressCorrectedBuff;
//      intPrint(((Display*)data)->bloodPressCorrectedBuff[j], 3, 0, 2);         //Systolic: should never be over 3 char
//      print("/", 3, 2);
//      intPrint(((Display*)data)->bloodPressCorrectedBuff[j+8], 5, 4, 2);         //Diastolic: should never be over 5 char
//      print("mm Hg", 9, 2);
//    }
//  
//    if( *m == TEMP){  // temperature
//      print("Temperature:", 0, 1);
//      fPrint(((Display*)data)->tempCorrectedBuff[j], 4, 0, 2);               //Temperature: should never be over 4 char
//      print("C", 4, 2);
//    }
//    
//    if( *m == HR){  // heart rate
//      print("Heart Rate:", 0, 1);
//      intPrint(((Display*)data)->pulseRateCorrectedBuff[j], 3, 0, 2);        //Heartrate: should never be over 3 char
//      print("BPM", 3, 2);
//    }
//  }
//   
//  if( *m == ANNUNCIATE){  // Annunciate Mode
//    print("Annunciate:", 0, 0);
//    volatile int t = *((Display*)data)->bloodPressCorrectedBuff;
//    intPrint(((Display*)data)->bloodPressCorrectedBuff[j], 3, 0, 1);         //Systolic: should never be over 3 char
//    print("/", 3, 1);
//    intPrint(((Display*)data)->bloodPressCorrectedBuff[j+8], 5, 4, 1);         //Diastolic: should never be over 5 char
//    print("mm Hg", 9, 1);
//
//    fPrint(((Display*)data)->tempCorrectedBuff[j], 4, 0, 2);               //Temperature: should never be over 4 char
//    print("C", 4, 2);
//    intPrint(((Display*)data)->pulseRateCorrectedBuff[j], 3, 6, 2);        //Heartrate: should never be over 3 char
//    print("BPM", 9, 2);
//
//    intPrint(*((Display*)data)->batteryState, 3, 13, 2);    //battery: should never be over 3 char
//  }
//*/
//}
//
//void annunciate(void* data) {
//  
///*    float t = (float)((WarningAlarm*)data)->tempRawBuff[j];
//    unsigned int s = ((WarningAlarm*)data)->bloodPressRawBuff[j];
//    float d = (float)((WarningAlarm*)data)->bloodPressRawBuff[j+8];
//    unsigned int h = ((WarningAlarm*)data)->pulseRateRawBuff[j];
//    short b = *(((WarningAlarm*)data)->batteryState);
//
//    t = 5 + (0.75*t);
//    s = 9 + (2 * s);
//    d = 6 + (1.5*d);
//    h = 8 + (3 * h);
//
//    if(b < 40 || h > 100 || h < 60 || t > 37.8 || t < 36.1 || 
//       s > 120 || s < 90 || d > 80 || d < 60 ){
//         
//      if (b < 40 && time % 30 == 0) { // three seconds have passed
//        if( GPIO_PORTF_DATA_R == ~(0x01)){ // LED is off
//          GPIO_PORTF_DATA_R |= 0x01; // Turn on the LED
//        }else{
//          GPIO_PORTF_DATA_R &= ~(0x01); // Turn off the LED
//        }
//      }
//  
//      if (h > 100 || h < 60) {
//        if(time % 20 == 0){ // two second have passed
//         if( GPIO_PORTF_DATA_R == ~(0x01)){ // LED is off
//            GPIO_PORTF_DATA_R |= 0x01; // Turn on the LED
//          }else{
//            GPIO_PORTF_DATA_R &= ~(0x01); // Turn off the LED
//          }
//        } 
//      }
//      
//      if (t > 37.8 || t < 36.1) {
//        if(time % 10 == 0){ // one second has passed
//         if( GPIO_PORTF_DATA_R == ~(0x01)){ // LED is off
//          GPIO_PORTF_DATA_R |= 0x01; // Turn on the LED
//          }else{
//            GPIO_PORTF_DATA_R &= ~(0x01); // Turn off the LED
//          }
//        } 
//      }
//  
//      if (s > 120 || s < 90 || d > 80 || d < 60) {
//        if(time % 5 == 0){ // one half-second has passed
//         if( GPIO_PORTF_DATA_R == ~(0x01)){ // LED is off
//          GPIO_PORTF_DATA_R |= 0x01; // Turn on the LED
//          }else{
//            GPIO_PORTF_DATA_R &= ~(0x01); // Turn off the LED
//          }
//        }
//        bpHigh = TRUE;
//      }else{
//        bpHigh = FALSE;
//      }
//  
//      if(s > 144 && bpHigh == TRUE){ // 20% over max systolic
//        if(time % 10 == 0){ // one second has passed
//         if( speakerOn == FALSE){ // speaker is off
//          PWMGenEnable(PWM_BASE, PWM_GEN_0); // Turn on the speaker
//          speakerOn = TRUE;
//          }else{
//            PWMGenDisable(PWM_BASE, PWM_GEN_0); // Turn off the speaker
//            speakerOn = FALSE;
//          }
//        }
//      }else{
//        PWMGenDisable(PWM_BASE, PWM_GEN_0); // Turn off the speaker
//      }
//    addFlags[6] = 1;            //add serial communication flag
//    }else{ // no warnings
//      GPIO_PORTF_DATA_R |= 0x01; // Turn on the LED
//    }*/
//}
//
//
//void status(void* data) {
//  if(time%100 == 0)
//	*(((Status*)data)->batteryState) -= 1;              //decrement battery by 1
//}
//
//void schedule(void* data) {
//        /*if(measureDelete != 0){
//          resetMeasureBufferAt(indexToDel);
//          measureDelete = 0;
//        }*/
//       /* for (int j = 0; j < NUMTASKS; j++){
//          if(addFlags[j] != 0){
//            addFlags[j] = 0;
//            llEnqueue(taskQueue,&tasks[j]);
//          }
//        }
//        if(time%10 == 0 && (time/10)%2 == 0){
//          addFlags[0] = 1; //flag measure
//          addFlags[3] = 1; //flag annunciate
//          addFlags[4] = 1; //flag status
//        }*/
//}
//
//void print(char* c, int hOffset, int vOffset) {                        // string, column, row
//  //RIT128x96x4StringDraw(c, hSpacing*(hOffset), vSpacing*(vOffset), 15);
//}
//
//void intPrint(int c, int size, int hOffset, int vOffset) {             // number, size of number,column, row
//  /*char dec[2];
//  dec[1] = '\0';
//  int rem = c;
//  for (int i = size - 1; i >= 0; i--) {
//    dec[0] = rem % 10 + '0';
//    rem = rem / 10;
//    RIT128x96x4StringDraw(dec, hSpacing*(hOffset + i), vSpacing*(vOffset), 15);
//  }*/
//}
//
////print of a float with one decimal
//void fPrint(float c, int size, int hOffset, int vOffset) {// number, size of number,column, row 
//  /*char dec[2];
//  dec[1] = '\0';
//  int rem = (int)c * 10;
//  for (int i = size - 1; i >= 0; i--) {
//    if (i == size - 2)
//      RIT128x96x4StringDraw(".", hSpacing*(hOffset + i), vSpacing*(vOffset), 15);
//    else {
//      dec[0] = rem % 10 + '0';
//      rem = rem / 10;
//      RIT128x96x4StringDraw(dec, hSpacing*(hOffset + i), vSpacing*(vOffset), 15);
//    }
//  }*/
//}
//
//void fillStructs(Measurements* m, ComputeData* c, Display* d, Status* s, WarningAlarm* w, Keypad* k, Communications* z) {
//
//  m->tempRawBuff = &tempRawBuff[0];                   //measure
//  m->bloodPressRawBuff = &bloodPressRawBuff[0];
//  m->pulseRateRawBuff = &pulseRateRawBuff[0];
//  m->reverseTemp = FALSE;
//  m->sysComplete = FALSE;
//  m->reversePulse = FALSE;
//  
//  c->tempRawBuff = &tempRawBuff[0];                            //compute
//  c->bloodPressRawBuff = &bloodPressRawBuff[0];
//  c->pulseRateRawBuff = &pulseRateRawBuff[0];
//  c->tempCorrectedBuff = &tempCorrectedBuff[0];
//  c->bloodPressCorrectedBuff = &bloodPressCorrectedBuff[0];
//  c->pulseRateCorrectedBuff = &pulseRateCorrectedBuff[0];
//  
//  d->tempCorrectedBuff = &tempCorrectedBuff[0];                //display
//  d->bloodPressCorrectedBuff = &bloodPressCorrectedBuff[0];
//  d->pulseRateCorrectedBuff = &pulseRateCorrectedBuff[0];
//  d->batteryState = &batteryState;
//  d->mode = &mode;
//  
//  s->batteryState = &batteryState;                    //status
//  
//  w->tempRawBuff = &tempRawBuff[0];                   //warningAlarm
//  w->bloodPressRawBuff = &bloodPressRawBuff[0];
//  w->pulseRateRawBuff = &pulseRateRawBuff[0];
//  w->batteryState = &batteryState;
//  
//  k->mode = &mode;
//  k->measurementSelection = &measurementSelection;
//  k->scroll = &scroll;
//  k->select = &select;
//  k->alarmAcknowledge = &alarmAcknowledge;
//  
//  z->tempCorrectedBuff = &tempCorrectedBuff[0];
//  z->bloodPressCorrectedBuff = &bloodPressCorrectedBuff[0];
//  z->pulseRateCorrectedBuff = &pulseRateCorrectedBuff[0];
//  z->batteryState  = &batteryState;
//}
//
//void fillBuffers() {
//  for(int i = 0; i < 8; i++) {
//    tempRawBuff[i] = 75;
//    bloodPressRawBuff[i] = 80;
//    bloodPressRawBuff[i+8] = 80;
//    pulseRateRawBuff[i] = 0;
//  }
//}
//
//void resetMeasureBufferAt(int j) {
//    tempRawBuff[j] = 75;
//    bloodPressRawBuff[j] = 80;
//    bloodPressRawBuff[j+8] = 80;
//    pulseRateRawBuff[j] = 0;
//}

/* Built in Functions */
void prvSetupHardware( void ){
  /* If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
    a workaround to allow the PLL to operate reliably. */
  if( DEVICE_IS_REVA2 ){
    SysCtlLDOSet( SYSCTL_LDO_2_75V );
  }
  
  /* Set the clocking to run from the PLL at 50 MHz */
  SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );
  
  /* 	Enable Port F for Ethernet LEDs
		LED0        Bit 3   Output
		LED1        Bit 2   Output */
  SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOF );
  GPIODirModeSet( GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3), GPIO_DIR_MODE_HW );
  GPIOPadConfigSet( GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3 ), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
  
  vParTestInitialise();
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void ){
  static xOLEDMessage xMessage = { "PASS" };
  static unsigned long ulTicksSinceLastDisplay = 0;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  
  /* Called from every tick interrupt.  Have enough ticks passed to make it
	time to perform our health status check again? */
  ulTicksSinceLastDisplay++;
  if( ulTicksSinceLastDisplay >= mainCHECK_DELAY ){
    ulTicksSinceLastDisplay = 0;
    
    /* Has an error been found in any task? */
    if( xAreGenericQueueTasksStillRunning() != pdTRUE ){
      xMessage.pcMessage = "ERROR IN GEN Q";
    }
    else if( xIsCreateTaskStillRunning() != pdTRUE ){
      xMessage.pcMessage = "ERROR IN CREATE";
    }
    else if( xAreIntegerMathsTaskStillRunning() != pdTRUE ){
      xMessage.pcMessage = "ERROR IN MATH";
    }
    else if( xAreIntQueueTasksStillRunning() != pdTRUE ){
      xMessage.pcMessage = "ERROR IN INT QUEUE";
    }
    else if( xAreBlockingQueuesStillRunning() != pdTRUE ){
      xMessage.pcMessage = "ERROR IN BLOCK Q";
    }
    else if( xAreBlockTimeTestTasksStillRunning() != pdTRUE ){
      xMessage.pcMessage = "ERROR IN BLOCK TIME";
    }
    else if( xAreSemaphoreTasksStillRunning() != pdTRUE ){
      xMessage.pcMessage = "ERROR IN SEMAPHORE";
    }
    else if( xArePollingQueuesStillRunning() != pdTRUE ){
      xMessage.pcMessage = "ERROR IN POLL Q";
    }
    else if( xAreQueuePeekTasksStillRunning() != pdTRUE ){
      xMessage.pcMessage = "ERROR IN PEEK Q";
    }
    else if( xAreRecursiveMutexTasksStillRunning() != pdTRUE ){
      xMessage.pcMessage = "ERROR IN REC MUTEX";
    }
    else if( xAreQueueSetTasksStillRunning() != pdPASS ){
      xMessage.pcMessage = "ERROR IN Q SET";
    }
    else if( xAreEventGroupTasksStillRunning() != pdTRUE ){
      xMessage.pcMessage = "ERROR IN EVNT GRP";
    }
    
    configASSERT( strcmp( ( const char * ) xMessage.pcMessage, "PASS" ) == 0 );
    
    /* Send the message to the OLED gatekeeper for display. */
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR( xOLEDQueue, &xMessage, &xHigherPriorityTaskWoken );
  }
  
  /* Write to a queue that is in use as part of the queue set demo to
	demonstrate using queue sets from an ISR. */
  vQueueSetAccessQueueSetFromISR();
  
  /* Call the event group ISR tests. */
  vPeriodicEventGroupsProcessing();
}
/*-----------------------------------------------------------*/

void vOLEDTask( void *pvParameters ){
  xOLEDMessage xMessage;
  unsigned long ulY, ulMaxY;
  static char cMessage[ mainMAX_MSG_LEN ];
  extern volatile unsigned long ulMaxJitter;
  const unsigned char *pucImage;
  
  /* Functions to access the OLED.  The one used depends on the dev kit
  being used. */
  void ( *vOLEDInit )( unsigned long ) = NULL;
  void ( *vOLEDStringDraw )( const char *, unsigned long, unsigned long, unsigned char ) = NULL;
  void ( *vOLEDImageDraw )( const unsigned char *, unsigned long, unsigned long, unsigned long, unsigned long ) = NULL;
  void ( *vOLEDClear )( void ) = NULL;
  
  /* Map the OLED access functions to the driver functions that are appropriate
	for the evaluation kit being used. */
  switch( HWREG( SYSCTL_DID1 ) & SYSCTL_DID1_PRTNO_MASK ){
    case SYSCTL_DID1_PRTNO_6965	:
    case SYSCTL_DID1_PRTNO_2965	:	
      vOLEDInit = OSRAM128x64x4Init;
      vOLEDStringDraw = OSRAM128x64x4StringDraw;
      vOLEDImageDraw = OSRAM128x64x4ImageDraw;
      vOLEDClear = OSRAM128x64x4Clear;
      ulMaxY = mainMAX_ROWS_64;
      pucImage = pucBasicBitmap;
      break;
  
    case SYSCTL_DID1_PRTNO_1968	:
    case SYSCTL_DID1_PRTNO_8962 :	
      vOLEDInit = RIT128x96x4Init;
      vOLEDStringDraw = RIT128x96x4StringDraw;
      vOLEDImageDraw = RIT128x96x4ImageDraw;
      vOLEDClear = RIT128x96x4Clear;
      ulMaxY = mainMAX_ROWS_96;
      pucImage = pucBasicBitmap;
      break;
  
    default:
      vOLEDInit = vFormike128x128x16Init;
      vOLEDStringDraw = vFormike128x128x16StringDraw;
      vOLEDImageDraw = vFormike128x128x16ImageDraw;
      vOLEDClear = vFormike128x128x16Clear;
      ulMaxY = mainMAX_ROWS_128;
      pucImage = pucGrLibBitmap;
      break;
  }
  
  ulY = ulMaxY;
  
  /* Initialise the OLED and display a startup message. */
  vOLEDInit( ulSSI_FREQUENCY );
  vOLEDStringDraw( "POWERED BY FreeRTOS", 0, 0, mainFULL_SCALE );
  //vOLEDImageDraw( pucImage, 0, mainCHARACTER_HEIGHT + 1, bmpBITMAP_WIDTH, bmpBITMAP_HEIGHT );
  
  for( ;; ){
    /* Wait for a message to arrive that requires displaying. */
    xQueueReceive( xOLEDQueue, &xMessage, portMAX_DELAY );
    
    /* Write the message on the next available row. */
    ulY += mainCHARACTER_HEIGHT;
    if( ulY >= ulMaxY ){
      ulY = mainCHARACTER_HEIGHT;
      vOLEDClear();
      vOLEDStringDraw( pcWelcomeMessage, 0, 0, mainFULL_SCALE );
    }
    
    /* Display the message along with the maximum jitter time from the
		high priority time test. */
    sprintf( cMessage, "%s [%uns]", xMessage.pcMessage, ulMaxJitter * mainNS_PER_CLOCK );
    vOLEDStringDraw( cMessage, 0, ulY, mainFULL_SCALE );
  }
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName ){
  ( void ) pxTask;
  ( void ) pcTaskName;
  
  for( ;; );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char *pcFile, unsigned long ulLine ){
  volatile unsigned long ulSetTo1InDebuggerToExit = 0;
  
  taskENTER_CRITICAL();{
    while( ulSetTo1InDebuggerToExit == 0 ){
      /* Nothing do do here.  Set the loop variable to a non zero value in
			the debugger to step out of this function to the point that caused
			the assertion. */
      ( void ) pcFile;
      ( void ) ulLine;
    }
  }
  taskEXIT_CRITICAL();
}
