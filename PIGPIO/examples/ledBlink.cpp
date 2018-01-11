#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include "irslinger.h"



int main(int argc, char *argv[])
{
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

    std::string code="00000";
    std::string bit1=std::to_string(std::rand(1));
    std::string bit2=std::to_string(std::rand(1));
    std::string bit3=std::to_string(std::rand(1));

    code=code+bit1+bit2+bit3;

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
	delay(1000);
    }
}		
