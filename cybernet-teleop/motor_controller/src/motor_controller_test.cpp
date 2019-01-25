#include <motor_controller/motor_controller.h>

int main(int argc, char** argv) {
	ros::init(argc,argv, "motor_controller_test");
	ros::NodeHandle nh;
	MotorController motorcontroller;
	while(ros::ok()) {
		//ROS_INFO("Setting motor 1 voltage = 0.2");
		motorcontroller.setVoltage(0,0.2);
	}
	return 0;
}