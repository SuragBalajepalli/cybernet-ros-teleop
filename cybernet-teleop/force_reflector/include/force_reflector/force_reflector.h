#ifndef FORCE_REFLECTOR_H
#define FORCE_REFLECTOR_H
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cstdlib>
#include <libphidget22/phidget22.h>
#include <motor_controller/motor_controller.h>

class ForceReflector {
protected:
	double RATE, JNT0_V_PER_N, JNT1_V_PER_N, JNT2_V_PER_N,
		JNT3_V_PER_NM , JNT4_V_PER_NM , JNT5_V_PER_NM ,
		JNT0_MIN , JNT0_MAX , JNT1_MIN , JNT1_MAX,
		JNT2_MIN , JNT2_MAX , JNT3_MIN , JNT3_MAX,
		JNT4_MIN , JNT4_MAX , JNT5_MIN , JNT5_MAX;
	std::string ft_sensor_topic;
public:
	ForceReflector(ros::NodeHandle* nh);
	void setDefaults();
	MotorController motor_controller;
};
#endif