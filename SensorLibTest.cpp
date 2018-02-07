#include <stdio.h>
#include <wiringPi.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "Libraries/SensorLib2.h"

int main()
{
    MUX *mux= initMUX();
    TOF *tof=newTOF(1,mux,7);

    TOF *tof2=newTOF(0,mux,3);

    printf("Initialized tof\n");

    while(1) {
        int distance = getDistance(tof);
	printf("Lrange : %i\n\n",distance);
	int distance2 = getDistance(tof2);
	printf("Srange : %d\n\n",distance2);
    }
}
