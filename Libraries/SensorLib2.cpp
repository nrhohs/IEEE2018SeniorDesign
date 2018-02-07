#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include "Adafruit_VL6180X.h"
#include "inc/vl53l0x_api.h"
#include "inc/vl53l0x_platform.h"
#include "inc/vl53l0x_def.h"
#include "RTIMULib.h"
#include "RTMath.h"
#include "SensorLib2.h"
#include "vl53l0x_setup.h"
#include <pigpio.h>
#include <sys/time.h>
#include <math.h>

//Private Helper prototypes
//I2C Expander
int getMUXStatus(MUX *);
void setMUXStatus(MUX *, int);
int getMUXInput(MUX *);
void setMUXInput(MUX *, int);

/* I2C Expander MUX */
/*
 * Initialize the MUX
 *
 * This function initializes the I2C Expander and returns
 * a MUX object
 *
 */
MUX *initMUX() {
    MUX *newMUX = (MUX *) malloc(sizeof(MUX));
    char buf[15];
    snprintf(buf,15,"/dev/i2c-1");
    newMUX->muxStatus = open(buf,O_RDWR);
    if (ioctl(newMUX->muxStatus,I2C_SLAVE,0x70)<0) {
	printf("MUX initialization failed\n");
	setMUXStatus(newMUX,-1);
	return newMUX;
    }
    else {
	setMUXStatus(newMUX,0);
	switchMUX(newMUX,0);
	printf("MUX initialization successfull\n");
	return newMUX;
    }
}

/*
 * Changes the active input of the I2C Expander
 * to the inputNo specified by user
 */
void switchMUX(MUX *mux, int inputNo) {
    if(!getMUXStatus(mux)) {
        mux->data_write[0] = 1<<inputNo;
        write(mux->muxStatus, mux->data_write, 1);
	setMUXInput(mux,inputNo);
    }
    else {
	printf("Cannot switch MUX, status error\n");
    }
}

/* VL6180X */
/*
 *
 * Initializer for the short range VL6180X  
 *
 */
SRANGE *initVL6180X(MUX *mux,int inputNo) {
    SRANGE *newSRANGE = (SRANGE *) malloc(sizeof(SRANGE));
    newSRANGE->vl = Adafruit_VL6180X();
    newSRANGE->mux=mux;
    newSRANGE->inputNo=inputNo;

    switchMUX(mux,inputNo);

    newSRANGE->vl.begin();
    return newSRANGE;
}

/*
 *
 * Gets the distance in mm for VL6180X
 *
 * Gets the distance from the short range TOF
 * specified by the user
 */
uint8_t getShortRange(SRANGE *srange) {
    switchMUX(srange->mux,srange->inputNo);
    srange->status=srange->vl.readRangeStatus();

    //if (srange->status==0)
	srange->range=srange->vl.readRange();
    /*else
	printf("Srange error %d cannot read range\n",srange->status);*/
    return srange->range;
}

void print_range_status(LRANGE *lrange) {
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;


    // if pRangingMeasurementData->RangeStatus != 0 then data is invalid
    RangeStatus = lrange->RangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    printf("Range Status: %i : %s\n", RangeStatus, buf);
}


VL53L0X_Error calibrateSingleMeasure(LRANGE *lrange)
{
    uint32_t refSpadCount;
    uint8_t isApertureSpads;uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(lrange->Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_StaticInit\n");
        lrange->Status = VL53L0X_StaticInit(lrange->pMyDevice);
	// Device Initialization
        print_pal_error(lrange->Status);
    }
    
    if(lrange->Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_PerformRefCalibration\n");
        lrange->Status = VL53L0X_PerformRefCalibration(lrange->pMyDevice,
        		&VhvSettings, &PhaseCal); // Device Initialization
        print_pal_error(lrange->Status);
    }

    if(lrange->Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_PerformRefSpadManagement\n");
        lrange->Status = VL53L0X_PerformRefSpadManagement(lrange->pMyDevice, &refSpadCount, &isApertureSpads);
        printf ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
        print_pal_error(lrange->Status);
    }

    // Enable/Disable Sigma and Signal check
	
    if (lrange->Status == VL53L0X_ERROR_NONE) {
        lrange->Status = VL53L0X_SetLimitCheckEnable(lrange->pMyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (lrange->Status == VL53L0X_ERROR_NONE) {
        lrange->Status = VL53L0X_SetLimitCheckEnable(lrange->pMyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (lrange->Status == VL53L0X_ERROR_NONE) {
        lrange->Status = VL53L0X_SetLimitCheckEnable(lrange->pMyDevice,
			VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    }

    if (lrange->Status == VL53L0X_ERROR_NONE) {
	lrange->Status = VL53L0X_SetLimitCheckEnable(lrange->pMyDevice,
			VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
			(FixPoint1616_t)(1.5*0.023*65536));
    }
			
	
    if (lrange->Status == VL53L0X_ERROR_NONE) {
        lrange->Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(lrange->pMyDevice, 200000);
	}
	
    if (lrange->Status == VL53L0X_ERROR_NONE) {
        lrange->Status = VL53L0X_SetVcselPulsePeriod(lrange->pMyDevice,
		        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    }
    if (lrange->Status == VL53L0X_ERROR_NONE) {
        lrange->Status = VL53L0X_SetVcselPulsePeriod(lrange->pMyDevice, 
		        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    }


    return lrange->Status;
}

VL53L0X_Error dataInitializeLR(LRANGE *lrange) {
    if (lrange->Status == VL53L0X_ERROR_NONE)
    {
    	printf("Call of VL53L0X_DataInit\n");
	lrange->Status = VL53L0X_DataInit(lrange->pMyDevice);
	print_pal_error(lrange->Status);
    }
    return lrange->Status;
}
     
LRANGE *initLongRange(MUX *mux, int inputNo) {
    LRANGE *lrange=(LRANGE *) malloc(sizeof(LRANGE));
    lrange->pMyDevice = &lrange->MyDevice;
    lrange->Status = VL53L0X_ERROR_NONE;
    lrange->mux=mux;
    lrange->inputNo=inputNo;

    switchMUX(mux,inputNo);

    //Initalize I2C comms
    lrange->pMyDevice->I2cDevAddr = 0x29;
    //Can choose between i2c-0 and ic2-1
    lrange->pMyDevice->fd = VL53L0X_i2c_init("/dev/i2c-1", lrange->pMyDevice->I2cDevAddr);
    if (lrange->MyDevice.fd<0) {
	lrange->Status = VL53L0X_ERROR_CONTROL_INTERFACE;
	printf("Failed to init on i2c\n");
    }
    printf("comms init\n");

    //Data initialization
    lrange->Status = dataInitializeLR(lrange);
    printf("lrange data initialized");

    //Calibrate for Single Range Measurement
    lrange->Status = calibrateSingleMeasure(lrange);
    printf("lrange calibrated\n");
    
    return lrange;
}

uint16_t getLongRange(LRANGE *lrange) {
    switchMUX(lrange->mux,lrange->inputNo);

    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
	lrange->Status = VL53L0X_PerformSingleRangingMeasurement(lrange->pMyDevice,&RangingMeasurementData);
	lrange->RangingMeasurementData=&RangingMeasurementData;
    	return RangingMeasurementData.RangeMilliMeter;
}

TOF *newTOF(int rangeType,MUX *mux,int input) {
    TOF *tof=(TOF *) malloc(sizeof(TOF));
    //rangeType 0 = short range
    // 		1 = long range
    if (rangeType==0)
    {
	SRANGE *srange=initVL6180X(mux,input);
	tof->srange=srange;
	tof->isLRANGE=rangeType;
	tof->mux=mux;
	tof->inputNo=input;
    }
    else if(rangeType==1)
    {
	LRANGE *lrange=initLongRange(mux,input);
	tof->lrange=lrange;
	tof->isLRANGE=rangeType;
	tof->mux=mux;
	tof->inputNo=input;
    }
    else {
    	printf("Invalid range type for TOF\n");
	return NULL;
    }
    return tof;
}

int isLRANGE(TOF *tof) {
    return tof->isLRANGE;
}

int getDistance(TOF *tof) {
    int distance;
    if (isLRANGE(tof)) {
	LRANGE *lrtof=tof->lrange;
	distance=getLongRange(lrtof);
    }
    else {
	//distance=tof->srange->vl.readRange();
	distance=getShortRange(tof->srange);
    }
    return distance;
}


RTIMU *imuInit() {
    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    //  This is an opportunity to manually override any settings before the call IMUInit

    //  set up IMU

    imu->IMUInit();

    //  this is a convenient place to change fusion parameters

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(false);
    return imu;
}

double getCurrImuRoll(RTIMU *imu) {
    RTIMU_DATA imuData = imu->getIMUData();
    RTVector3 vec=imuData.fusionPose;
    double roll=vec.x() * RTMATH_RAD_TO_DEGREE;
    printf("current:%f \n",roll);
    return roll;
}

double getCurrImuPitch(RTIMU *imu) {
    RTIMU_DATA imuData = imu->getIMUData();
    RTVector3 vec=imuData.fusionPose;
    double pitch=vec.x() * RTMATH_RAD_TO_DEGREE;
    printf("current:%f \n",pitch);
    return pitch;
}

double getCurrImuYaw(RTIMU *imu) {
    RTIMU_DATA imuData = imu->getIMUData();
    RTVector3 vec=imuData.fusionPose;
    double yaw=vec.z() * RTMATH_RAD_TO_DEGREE; 
    printf("current:%f  \n",yaw);
    return yaw;
}

//Private Helper methods
int getMUXStatus(MUX *mux) {
    return mux->initVal;
}
void setMUXStatus(MUX *mux, int val) {
    mux->initVal=val;
}
int getMUXInput(MUX *mux) {
    return mux->activeInput;
}
void setMUXInput(MUX *mux, int val) {
    mux->activeInput=val;
}


//Store 0 or 1 as new command with time inverval
cmd *addCmd(int initVal, unsigned long initUSec) {
    cmd *newCmd;
    newCmd = (cmd *)malloc(sizeof(cmd));
    
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
    char *strcode=(char *)malloc(sizeof(char)*8);
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

int routeread()
{
    if (gpioInitialise() < 0) return 1;       //initialize the gpio
    
    gpioSetMode(16, PI_INPUT);                //Set pin mode
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
	return(decodesStr);

}
