#include <stdio.h>                                                                        
#include <wiringPi.h>                                                             
#include <mcp23017.h>                                                          
#include <lcd.h>                                                               
#include <softPwm.h> 
int main()
{
	long value=0;
	int rand_num;	
	int value_blue;
	int value_red;
	int value_green;
	int display,i,count;                                                   
	wiringPiSetup();                                                 
	mcp23017Setup (100, 0x20);                                                
	printf ("Raspberry Pi - MCP23017 Test\n");
	for(i=0;i<16;i++)
	pinMode(100+i,OUTPUT);                                                   
	digitalWrite(101,0);                                                       
	display=lcdInit(2,16,4,100,102,103,104,105,106,0,0,0,0);                   
	lcdHome(display);                                                           
	lcdClear(display);                                                       
	lcdPosition(display,0,0);                                                   
	lcdPuts(display,"Hello World");                                                      
	lcdPosition(display,0,1);                                                  
	lcdPuts(display,"www.52pi.net");                            				
	pinMode(0, OUTPUT);                                                        
	pinMode(2, OUTPUT);                                                        
	pinMode(3, OUTPUT);                                                         
	softPwmCreate (3, 50, 100);
	softPwmCreate (2, 50, 100);
	softPwmCreate (0, 50, 100);
while(1)
{       
    delay(200);                                                       
    value_red=(value<100)?value:0;
	value_green=(value>100&&value<200)?(value-100):0;
	value_blue=(value>200)?(value-200):0;                                  
	if(value>300)
	{
		value_red=rand()%100;
		value_green=rand()%100;
		value_blue=rand()%100;
	}
	softPwmWrite (3,value_red);
	softPwmWrite (2,value_green);
	softPwmWrite (0,value_blue);
	value++;
	if(value>900)
	{
		value=0;
	}
		
	lcdPosition(display,13,1);
	lcdPrintf(display,"%d",value);		
	}
}
