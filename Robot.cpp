#include "robot.h"

Robot::Robot() {
	//Initializes MUX
	mux = initMUX();

	//Initializes TOFs
	//Let's create an array of TOF's so that we can pass them to a function	
	TOF *tof0 = newTOF(0, mux, 0);	//front facing SR
	TOF *tof1 = newTOF(0, mux, 1);	//right facing SR (front)
	TOF *tof2 = newTOF(0, mux, 2);	//right facing SR (back)
	TOF *tof3 = newTOF(0, mux, 3);	//back facing SR
	TOF *tof4 = newTOF(0, mux, 4);	//left facing SR  (back)
	TOF *tof5 = newTOF(0, mux, 5);	//left facing SR  (front)
	TOF *tof6 = newTOF(1, mux, 6);	//right facing LR (box TOF)
	TOF *tof7 = newTOF(1, mux, 7);	//front facing LR (dropoff TOF)

	TOFs.push_back(tof0);
	TOFs.push_back(tof1);
	TOFs.push_back(tof2);
	TOFs.push_back(tof3);
	TOFs.push_back(tof4);
	TOFs.push_back(tof5);
	TOFs.push_back(tof6);
	TOFs.push_back(tof7);

									//Initializes IMU
	RTIMU *imu = imuInit();
}