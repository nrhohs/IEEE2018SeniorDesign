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

int main(int argc, char *argv[])
{
    Pi_Rec_t *rec;
    if (gpioInitialise() < 0) return 1;       //initialize the gpio
    
    gpioSetMode(23, PI_INPUT);                //Set pin mode
    
    while (1) {
        int value = 1;                            //Set value to 1, receiver outputs 0 if pulse
        struct timeval timeStart, timeEnd;
        struct Command cmdRecord[1000];
        int ii = 0;
        
        while (value)
            value = gpioRead(23);
        
        #Get the time when pulse is detected (0 input)
        gettimeofday(&timeStart,NULL);
        
        int previousVal = 0;
        
        while (1 && ii < 1000) {
            if value != previousVal {
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
        
        #Print results
        printf("---------------------Start-------------------\n");
        int j;
        for (j = 0;j < ii; j++) {
            printf("%d\t\t%lu\n",cmdRecord[j]->cmdVal, cmdRecord[j]->cmdUSec);
        }
        printf("----------------------End--------------------\n");
    }
}

#Store 0 or 1 as new command with time inverval
cmd *addCmd(int initVal, unsigned long initUSec) {
    cmd *newCmd;
    newCmd = malloc(sizeof(cmd));
    
    newCmd->cmdVal = initVal;
    newCmd->cmdUSec = initUSec;
    
    return newCmd;
}
        
    
