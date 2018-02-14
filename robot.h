#ifndef ROBOT_H
#define ROBOT_H
#include "robot.h"
#include "Libraries/SensorLib2.h"

#include <vector>

class Robot
{
	//need to discuss what should be public and private member variables
public:
	Robot();

	//Initializes MUX
	MUX *mux;
	std::vector<TOF*> TOFs;
	RTIMU *imu;
private:

};



#endif // !ROBOT_H
