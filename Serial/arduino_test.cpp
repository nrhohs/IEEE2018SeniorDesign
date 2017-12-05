#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "rs232.h"

#define BUF_SIZE 8 

int main()
{
  int cport_nr=24; /* /dev/ACM0*/
  int bdrate=9600; /* 9600 baud */

  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit

  unsigned char str_send[BUF_SIZE];
  unsigned char str_recv[BUF_SIZE]; // recv data buffer
  str_send[0] = 0b00101000;
  str_send[1] = 100;
  int i=0;
  
  if(RS232_OpenComport(cport_nr, bdrate, mode))
  {
    printf("Can not open comport\n");
    return(0);
  }

  usleep(2000000);  /* waits 2000ms for stable condition */
  while(1)
  {


  if(i>10) {
    str_send[0] = 0b00110000;
  }

    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
	printf("Sent to Arduino: '%X %X'\n", str_send[0],str_send[1]);
	usleep(1000000);  /* waits for reply 1000ms */
//	int n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
//	if(n > 0){
//      str_recv[n] = 0;   /* always put a "null" at the end of a string! */
//      printf("Received %i bytes: '%2X  %3d'\n", n, str_recv[0],str_recv[1]);
//	}
    usleep(1000000);  /* sleep for 1 Second */
  i++;
  }
  return(0);
}

