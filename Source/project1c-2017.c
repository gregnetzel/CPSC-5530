#include <stdio.h>

//  function prototypes

//  get data from the user
void getData(int* aValuePtr);

void main (void)
{
	//  declare a shared variable and a pointer to it
	int myValue;
	int* myPtr = &myValue;		//  let myPtr point to myValue

	//  get data from the user
	getData(myPtr);

	//  display the data as a character
	printf("The data is: %c \n", *myPtr);

}

//  prompt the user for some data and return it through a shared 
//  variable pointed to by valuePtr
//
//  inputs:  	pointer to a container in which to place the data
//  outputs:	none
//  function:	the routine accepts a pointer to a container in which to store data from a user,
//				it prompts for the data, accepts the data, displays it, and returns

void getData(int* valuePtr)
{
	//  declare a temp place to store the data
	int tempValue;

	//  let valuePtr point to it
	valuePtr = &tempValue;

	//  prompt for data
	printf("Please enter a single digit between 0-9 \n");

	//  get the data
	*valuePtr = getchar();

	//  display its value as a character 
	printf("The data is: %c \n", *valuePtr);

	return;

}
