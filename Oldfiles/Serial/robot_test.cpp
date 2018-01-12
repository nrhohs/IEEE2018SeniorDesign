#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "rs232.h"
#include <wiringPi.h>
#include <mcp23017.h>
#include <lcd.h>
#include <softPwm.h>


#define BUF_SIZE 8 

int main()
{
  int display, j;
//initialize LCD
  wiringPiSetup();
  mcp23017Setup (100, 0x20);
  for(j=0;j<16;j++)
    pinMode(100+j,OUTPUT);
  digitalWrite(101,0);
  display=lcdInit(2,16,4,100,102,103,104,105,106,0,0,0,0);
  lcdHome(display);
  lcdClear(display);
  lcdPosition(display,0,0);
  pinMode(0, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  softPwmCreate(3, 50, 100);
  softPwmCreate(2, 50, 100);
  softPwmCreate(0, 50, 100);
  softPwmWrite(3,100);
  softPwmWrite(2,0);
  softPwmWrite(0,0);



  int cport_nr=24; /* /dev/ACM0*/
  int bdrate=9600; /* 9600 baud */

  int i=0;
  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit

  unsigned char str_send[BUF_SIZE];
  str_send[0] = 0;
  str_send[1] = 30;
  
  if(RS232_OpenComport(cport_nr, bdrate, mode))
  {
    printf("Can not open comport\n");
    return(0);
  }

  usleep(2000000);  /* waits 2000ms for stable condition */
  while(1)
  {


  if(i<3) {
    str_send[0] = 1;
  }
  else if(i<4) {
    str_send[0] = 0;
  }
  else if(i<7) {
    str_send[0] = 3;
  }
  else if(i<8) {
    str_send[0] = 0;
  }
  else if(i<11) {
    str_send[0] = 2;
  }
  else if(i<12) {
    str_send[0] = 0;
  }
  else if(i<16) {
    str_send[0] = 4;
  }
  else if(i<17) {
    str_send[0] = 0;
  }
  else {
    i=-1;
  }




    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
	printf("Sent to Arduino: '%3d %3d'\n", str_send[0],str_send[1]);
        lcdPosition(display,0,0);
        lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    usleep(1000000);  /* sleep for 1 Second */
  i++;
  }
  return(0);
}

