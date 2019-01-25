//Not a very useful class, would be better as a node

#include <force_reflector/force_reflector.h>

ForceReflector::ForceReflector(ros::NodeHandle* nh) {
	setDefaults();
	
	//ros::Subscriber force_sub = nh.subscribe(ft_sensor_topic,1,forceCb);
	if(!nh->getParam("/joint0/v_per_n", JNT0_V_PER_N)) {
		ROS_INFO("Setting default value for joint0 voltage per newton: %f", JNT0_V_PER_N);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT0_V_PER_N);}
	if(!nh->getParam("/joint1/v_per_n", JNT1_V_PER_N)) {
		ROS_INFO("Setting default value for joint0 voltage per newton: %f", JNT1_V_PER_N);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT1_V_PER_N);}
	if(!nh->getParam("/joint2/v_per_n", JNT2_V_PER_N)) {
		ROS_INFO("Setting default value for joint0 voltage per newton: %f", JNT2_V_PER_N);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT2_V_PER_N);}
	if(!nh->getParam("/joint3/v_per_n", JNT3_V_PER_NM)) {
		ROS_INFO("Setting default value for joint0 voltage per newton: %f", JNT3_V_PER_NM);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT3_V_PER_NM);}
	if(!nh->getParam("/joint4/v_per_n", JNT4_V_PER_NM)) {
		ROS_INFO("Setting default value for joint0 voltage per newton: %f", JNT4_V_PER_NM);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT4_V_PER_NM);}
	if(!nh->getParam("/joint5/v_per_n", JNT5_V_PER_NM)) {
		ROS_INFO("Setting default value for joint0 voltage per newton: %f", JNT5_V_PER_NM);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT5_V_PER_NM);}
	if(!nh->getParam("/rate", RATE)) {
		ROS_INFO("default rate set at %f Hz", RATE);
	}
	else {ROS_INFO("Rate set from param sever %f", RATE);}
}

void ForceReflector::setDefaults() {
	RATE = 100.0;
	JNT0_V_PER_N = 1.0;
	JNT1_V_PER_N = 1.0;
	JNT2_V_PER_N = 1.0;
	JNT3_V_PER_NM = 1.0;
	JNT4_V_PER_NM = 1.0;
	JNT5_V_PER_NM = 1.0;
	JNT0_MIN = -2.0;
	JNT0_MAX = 2.0;
	JNT1_MIN = -2.0;
	JNT1_MAX = 2.0;
	JNT2_MIN = -2.0;
	JNT2_MAX = 2.0;
	JNT3_MIN = -2.0;
	JNT3_MAX = 2.0;
	JNT4_MIN = -2.0;
	JNT4_MAX = 2.0;
	JNT5_MIN = -2.0;
	JNT5_MAX = 2.0;
	ft_sensor_topic = "test_force_torque_sensor";
}