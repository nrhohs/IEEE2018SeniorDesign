#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "rs232.h"

#define BUF_SIZE 128

int main() {
  int i=0;
  int cport_nr=16;
  int brate=57600;

  char mode[] = {'8','N','1',0};
  char str_sent[2][BUF_SIZE];
  unsigned char str_recv[BUF_SIZE]
  strcpy(str_send[0], "This is a test string.");
  strcpy(str_sent[1], "This is another test string.");

  if(RS232_OpenComport(cport_nr, bdrate, mode)){
    printf("Can not open comport\n");
    return 0;
  }
  
  usleap(2000000);
  while(1) {
    RS232_cputs(cport_nr, str_sent[i]);
    printf("Sent to Arduino: '%s'\n", str_send[i];
    usleep(1000000);
    int n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
    if(n>0) {
      str_recv[n] = 0;
      printf("Received %i bytes: '%s'\n", n, (char *)str_recv);
    }
    i++;
    i %= 2;
    usleep(5000000);
  }
  return 0;
}




