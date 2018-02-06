#include <stdio.h>
#include <wiringPi.h>
#include <stdint.h>
#include <unistd.h>
#include "Libraries/navigation.h"
#include "Libraries/SensorLib2.h"

int main()
{
    //Initialize MUX
    MUX *mux = initMUX();

    	//TOF *tof0 = newTOF(1,mux,0);
	TOF *tof1 = newTOF(0,mux,2);
	printf("Is long range : %d\n",tof1->isLRANGE);
	printf("Input No. %d\n",tof1->inputNo);
	printf("Status %d\n",tof1->srange->vl->status);

    printf("Initalization done..\n");
    //while(1)
    //{
	//int lDistance=getDistance(tof0);
	//printf("Lrange : %d mm",lDistance);
	int sDistance=getDistance(tof1);
	printf("%3d\t, ",tof1->srange->range);
	printf("Srange : %d mm\n",sDistance);
    //}
}
