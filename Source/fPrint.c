#define hSpacing 8
#define vSpacing 12

void delay(unsigned long* aValue){
    volatile unsigned long i = 0;
    volatile int j = 0;
    for (i = *aValue; i > 0; i--){
        for (j = 0; j < 100000; j++);
    }
    return;
}

void fPrint(char c, int offset, unsigned long* delay1){
  char myData[3];
  myData[0] = c;              
  myData[1] = '\0';
  RIT128x96x4StringDraw(myData, hSpacing*(offset), vSpacing, 15);
  delay(delay1);
}