#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "rs232.h"

#define BUF_SIZE  16

int main()
{
  int cport_nr=24; /* /dev/ttyUSB0 */
  int bdrate=9600; /* 9600 baud */

  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit

  char str_send[BUF_SIZE];
  unsigned char str_recv[BUF_SIZE]; // recv data buffer
  uint16_t val = 0b11111111;
  str_send[0] = strtol("01010110", (char **)NULL, 2);
  str_send[1] = strtol("11111111", (char **)NULL, 2);

  
  if(RS232_OpenComport(cport_nr, bdrate, mode))
  {
    printf("Can not open comport\n");
    return(0);
  }

  usleep(2000000);  /* waits 2000ms for stable condition */
  while(1)
  {
    RS232_cputs(cport_nr, str_send);// sends string on serial
	printf("Sent to Arduino: '%s'\n", str_send);
	usleep(1000000);  /* waits for reply 1000ms */
	int n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
	if(n > 0){
      str_recv[n] = 0;   /* always put a "null" at the end of a string! */
      printf("Received %i bytes: '%3d  %3d'\n", n, str_recv[0],str_recv[1]);
	}
    usleep(1000000);  /* sleep for 1 Second */
  }
  return(0);
}

