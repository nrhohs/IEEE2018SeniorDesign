/********************************************************
					Navigation Library
This library is meant to be a set of intuitive navigation
functions that can be called by the main Robot.cpp file.
The purpose of this library is to make it easier to tweak
how the robot traverses the arena.
********************************************************/

void stop(int runtime);
void stop_readIMU();
void stop_readTOF();

void fwd_timed(unsigned char speed, int runtime);
void fwd_waitOnTOF();
void fwd_waidOnIMU();

void bwd_timed(unsigned char speed, int runtime);
void bwd_waitOnTOF();
void bwd_waitOnIMU();

void strafeRight_timed(unsigned char speed, int runtime);
void strafeRight_waitOnTOF();
void strafeRight_waitOnIMU();

void strafeLeft_timed(unsigned char speed, int runtime);
void strafeLeft_waitOnTOF();
void strafeLeft_waitOnIMU();

void turnRight_timed(unsigned char speed, int runtime);
void turnRight_waitOnTOF();
void turnRight_waitOnIMU();

void turnLeft_timed(unsigned char speed, int runtime);
void turnLeft_waitOnTOF();
void turnLeft_waitOnIMU();

void wallFollow();