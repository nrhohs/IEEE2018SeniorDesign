#ifndef __LEDREAD_H__
#define __LEDREAD_H__


#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>
#include <math.h>
#include <sys/time.h>


typedef struct Command cmd;

//Creates the units of the IR signal (cmds - "0" or "1")
cmd *addCmd(int initVal, unsigned long initUSec);

//Converts binary string to an int val
int binaryToDecimal(const char *);

//Actually decodes an array of cmds to the binary string
char *decodeSignal(cmd **cmdArr,int maxToggles); 

//Public function to call that reads the IR signal and spits out int val of the code
int ledRead();

#endif
