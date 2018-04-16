/********************************************************
					Navigation Library
This library is meant to be a set of intuitive navigation
functions that can be called by the main Robot.cpp file.
The purpose of this library is to make it easier to tweak
how the robot traverses the arena.
********************************************************/

#include "SensorLib2.h"

#define CPORT_NR 24

void waitOnTOF(TOF *,int target=100);
void waitOnIMU(RTIMU *,char,double,bool);
void waitTOFwIMU(unsigned char speed,RTIMU *imu,double IMUtarget,TOF *sensor,int TOFtarget,int display);
void stop(int runtime, int display);
int stop_readTOF(TOF *sensor, int display);

void fwd_timed(unsigned char speed, int runtime, int display);
void fwd_waitOnTOF(unsigned char speed, TOF *sensor, int display, int distance=100);
void fwd_waitOnIMU(unsigned char speed, RTIMU *imu, char axis, double target, int display);
void fwd_waitTOFwIMU(unsigned char speed,RTIMU *imu,double IMUtarget,TOF *sensor,int TOFtarget,int display, int turnSpeed);
void fwd_wallFollow(unsigned char speed, RTIMU *imu, TOF *sensor, TOF *tof2, double angle, int TOFtarget, int display);

void bwd_timed(unsigned char speed, int runtime, int display);
void bwd_waitOnTOF(unsigned char speed, TOF *sensor, int display, int distance = 100);
void bwd_waitOnIMU(unsigned char speed, RTIMU *imu, char axis, double target, int display);
void bwd_waitTOFwIMU(unsigned char speed,RTIMU *imu,double IMUtarget,TOF *sensor,int TOFtarget,int display, int turnSpeed);
void bwd_wallFollow(unsigned char speed, RTIMU *imu, TOF *sensor, TOF *tof2, double angle, int TOFtarget, int display);

void bwdArcRight_waitOnTOF(unsigned char speed, unsigned char offset, TOF *sensor, int display, int distance = 100);

void strafeRight_timed(unsigned char speed, int runtime, int display);
void strafeRight_waitOnTOF(unsigned char speed, TOF *sensor, int display, int distance = 100);
void strafeRight_waitOnIMU(unsigned char speed, RTIMU *imu, char axis, double target, int display);

void strafeLeft_timed(unsigned char speed, int runtime, int display);
void strafeLeft_waitOnTOF(unsigned char speed, TOF *sensor, int display, int distance = 110);
void strafeLeft_waitOnIMU(unsigned char speed, RTIMU *imu, char axis, double target, int display);
void strafeLeft_wallFollow(unsigned char speed, RTIMU *imu, TOF *sensor, TOF *tof2, double angle, int TOFtarget, int display);
void strafeLeft_wallFollow_d(unsigned char speed, RTIMU *imu, TOF *sensor, TOF *tof2, double angle, int TOFtarget, int display);

void diagonal_fwdLeft_waitOnTOF(unsigned char speed, TOF *sensor, int display, int distance = 110);
void diagonal_fwdRight_waitOnTOF(unsigned char speed, TOF *sensor, int display, int distance = 110);
void diagonal_bwdRight_waitOnTOF(unsigned char speed, TOF *sensor, int display, int distance = 110);
void diagonal_bwdLeft_waitOnTOF(unsigned char speed, TOF *sensor, int display, int distance = 110);

void turnRight_timed(unsigned char speed, int runtime, int display);
void turnRight_waitOnIMU(unsigned char speed, RTIMU *imu, double targetYaw, int display, bool isAbsolute);
void turnRight_waitOnIMU_F(unsigned char speed, RTIMU *imu, double targetYaw, int display, bool isAbsolute);

void turnLeft_timed(unsigned char speed, int runtime, int display);
void turnLeft_waitOnIMU(unsigned char speed, RTIMU *imu, double targetYaw, int display, bool isAbsolute);
void turnLeft_waitOnIMU_B(unsigned char speed, RTIMU *imu, double targetYaw, int display, bool isAbsolute);

void sendCommand(unsigned char command, unsigned char speed, int display);
void sendCommandNoDisplay(unsigned char command, unsigned char speed);
void stopExit();

void turnBackLeft_waitOnIMU(unsigned char speed, RTIMU *imu, double targetYaw, int display, bool isAbsolute);
void turnBackRight_waitOnIMU(unsigned char speed, RTIMU *imu, double targetYaw, int display, bool isAbsolute);

void turn_IMUcorrection(RTIMU *imu, double angle, int display);

void turnToFlag(RTIMU *imu, double angle, int display);
void driveToTOF(unsigned char speed,TOF *tof,int display,int target,bool dir);
void strafeToTOF(unsigned char speed,TOF *tof,int display,int target);

void drive_PID(unsigned char speed, TOF *tof, int display, int target);

void wallFollowDrive(unsigned char speed, unsigned char direction, TOF *face, TOF *sideOne, TOF *sideTwo, RTIMU *imu, int display);
void wallFollowStrafe(unsigned char speed, unsigned char direction, TOF *face, TOF *sideOne, TOF *sideTwo, RTIMU *imu, int display);
