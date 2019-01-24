#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H      


#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cstdlib>
#include <libphidget22/phidget22.h>

class MotorController {
public:
	
	MotorController(ros::NodeHandle& nh);
	~MotorController();
	//main function to be used:
	bool setVoltage(int motor_index,double voltage);
	//accessors for sn and channels and others
	long get_device_1_sn();
	long get_device_2_sn();
	bool set_device_1_sn(long sn);
	bool set_device_2_sn(long sn);
	bool set_timeout(double timeout);
protected:
	long device_1_sn = 525557;
	long device_2_sn = 525557;
	double timeout = 5000;
	std::vector<PhidgetVoltageOutputHandle> Vout_handles;
	PhidgetReturnCode prc;
};

#endif