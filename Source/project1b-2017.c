#include <stdio.h>

//  this is a simple routine that demonstrates how to fill and diaplay an array of characters

void main(void)
{
	int i = 0;									//  declare a working variable

	char myArray[5];							//  declare a character array

	for (i = 0; i <= 5; i++)					//  fill array with characters
	{
		//  fill with the ascii characters A..F
		//  65 is the ascii value for A

		myArray[i]= 65+i;
	}

	for (i = 0; i <= 5; i++)					//  display the array
	{
		printf("%c \n", myArray[i]);
	}

	printf("\n");

	return;
}
