/********************************************************
					Navigation Library
This library is meant to be a set of intuitive navigation
functions that can be called by the main Robot.cpp file.
The purpose of this library is to make it easier to tweak
how the robot traverses the arena.
********************************************************/

void stop(int runtime, int display);
void stop_readTOF(TOF *sensor, int display);

void fwd_timed(unsigned char speed, int runtime, int display);
void fwd_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void fwd_waidOnIMU(unsigned char speed, IMU *imu, int display);

void bwd_timed(unsigned char speed, int runtime, int display);
void bwd_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void bwd_waitOnIMU(unsigned char speed, IMU *imu, int display);

void strafeRight_timed(unsigned char speed, int runtime, int display);
void strafeRight_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void strafeRight_waitOnIMU(unsigned char speed, IMU *imu, int display);

void strafeLeft_timed(unsigned char speed, int runtime, int display);
void strafeLeft_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void strafeLeft_waitOnIMU(unsigned char speed, IMU *imu, int display);

void turnRight_timed(unsigned char speed, int runtime, int display);
void turnRight_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void turnRight_waitOnIMU(unsigned char speed, IMU *imu, int display);

void turnLeft_timed(unsigned char speed, int runtime, int display);
void turnLeft_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void turnLeft_waitOnIMU(unsigned char speed, IMU *imu, int display);

void wallFollowDrive(unsigned char speed, unsigned char direction, TOF *face, TOF *sideOne, TOF *sideTwo, IMU *imu, int display);
void wallFollowStrafe(unsigned char speed, unsigned char direction, TOF *face, TOF *sideOne, TOF *sideTwo, IMU *imu, int display);