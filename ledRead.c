#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>
#include <math.h>
#include <sys/time.h>


struct Command
{
    int cmdVal;
    unsigned long cmdUSec;
};

typedef struct Command cmd;

cmd *addCmd(int initVal, unsigned long initUSec);
int binaryToDecimal(const char *);
char *decodeSignal(cmd **cmdArr,int maxToggles); 

int main(int argc, char *argv[])
{
    if (gpioInitialise() < 0) return 1;       //initialize the gpio
    
    gpioSetMode(23, PI_INPUT);                //Set pin mode
    int maxToggles=40;
    
        int value = 1;                            //Set value to 1, receiver outputs 0 if pulse
        struct timeval timeStart, timeEnd;
        cmd *cmdRecord[maxToggles];
        int ii = 0;
        
        while (value)
            value = gpioRead(23);
        
        //Get the time when pulse is detected (0 input)
        gettimeofday(&timeStart,NULL);
        
        int previousVal = 0;
        
        while (1 && ii < maxToggles) {
            if (value != previousVal) {
                gettimeofday(&timeEnd,NULL);
                unsigned long uSecStart = 1000000 * timeStart.tv_sec + timeStart.tv_usec;
                unsigned long uSecEnd = 1000000 * timeEnd.tv_sec + timeEnd.tv_usec;
                unsigned long pulseLength = uSecEnd - uSecStart;
                
                gettimeofday(&timeStart,NULL);
                cmd *newCmd = addCmd(previousVal, pulseLength);
                cmdRecord[ii] = newCmd;
                ii++;
            }
            
            previousVal = value;
            value = gpioRead(23);
        }

	char *codeStr = decodeSignal(cmdRecord,maxToggles);
	printf("Code string is ");
	int k;
	for (k=0;k<8;k++)
	    printf("%c",codeStr[k]);
	printf("\n");

   	int decodesStr=binaryToDecimal(codeStr);
	printf("%d\n",decodesStr);
}

//Store 0 or 1 as new command with time inverval
cmd *addCmd(int initVal, unsigned long initUSec) {
    cmd *newCmd;
    newCmd = malloc(sizeof(cmd));
    
    newCmd->cmdVal = initVal;
    newCmd->cmdUSec = initUSec;
    
    return newCmd;
}

//Convert Binary to Decimal
int binaryToDecimal(const char *str) {
    return (int) strtol(str, NULL, 2);
}


//Decode raw signal for coDE
char *decodeSignal(cmd **cmdArr,int maxToggles) {
    char *strcode=malloc(sizeof(char)*8);
    char c;
    int j;
    int k=0;
    	for (j=0;j < maxToggles; j++) {
            if (cmdArr[j]->cmdUSec > 8000) j++;
	    else if (cmdArr[j]->cmdUSec > 3500) j++;
	    else if (cmdArr[j]->cmdVal == 1 && cmdArr[j]->cmdUSec > 1500) {
		c='1';
		strcode[k]=c;
		k++;
	    }
	    else if (cmdArr[j]->cmdVal == 1 && cmdArr[j]->cmdUSec < 1500) {
		c='0';
		strcode[k]=c;
		k++;
	    }

	    if (k>=8)
		break;
	} 
    return strcode;
}
