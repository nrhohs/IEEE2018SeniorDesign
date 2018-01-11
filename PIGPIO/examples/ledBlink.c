#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include "irslinger.h"
#include <unistd.h>
#include <time.h>

unsigned int intToBin(int k)
{
    return (k == 0 || k == 1 ? k : ((k % 2) + 10 * intToBin(k/2)));
} 

int main(int argc, char *argv[])
{
    time_t t;				//time seed for random
    uint32_t outPin=22;			//BCM pin for the LED for 
    int frequency = 38000;		//Frequency of IR in kHz
    double dutyCycle = 0.5;		//Duty cycle for IR
					//LED uses 50% of cycle
    
    int startingBurst = 9000;	//Duration of start pulse (us)
    int startGap = 4500;	//Duration of gap (us)
    int onePulse = 562;			//Pulse duration for a one (us)
    int zeroPulse = 562;		//Pulse duration for a zero (us) 
    int oneGap = 2250;			//Gap duration for a one (us)
    int zeroGap = 1120;			//Gap duration for a zero (us)
    int sendTrailPulse = 1;		//Sends pulse to end message
					//0 means don't send	
   
    srand((unsigned) time(&t));

    int bits=rand() % 7;
    char *code;
    
    switch (bits) {
    	case 0:
	   code="00000000";
	   break;
	case 1:
	   code="00000001";
	   break;
	case 2:
	   code="00000010"; 
	   break;
	case 3:
	   code="00000011";
	   break;
	case 4:
	   code="00000100";
	   break;
	case 5:
	   code="00000101";
	   break;
	case 6:
	   code="00000110";
	   break;
	case 7:
	   code="00000111";
	   break;
    }

    printf("\nCode Transmitting...\n");
    printf("Code is %d\n",bits);


    while (1) {
	int result = irSling(
		outPin,
		frequency,
		dutyCycle,
		startingBurst,
		startGap,
		onePulse,
		zeroPulse,
		oneGap,
		zeroGap,
		sendTrailPulse,
		code);			//Randomly generated 3 bit code 

	printf("Result:%d\n",result);	
	usleep(1000);
    }
}		
