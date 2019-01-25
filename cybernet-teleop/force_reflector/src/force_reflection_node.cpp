#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cstdlib>
#include <libphidget22/phidget22.h>
#include <motor_controller/motor_controller.h>


double RATE = 100.0, JNT0_V_PER_N = 1.0, JNT1_V_PER_N = 1.0, JNT2_V_PER_N = 1.0,
		JNT3_V_PER_NM = 1.0, JNT4_V_PER_NM = 1.0, JNT5_V_PER_NM = 1.0,
		JNT0_MIN = -2.0, JNT0_MAX = 2.0, JNT1_MIN = -2.0, JNT1_MAX = 2.0,
		JNT2_MIN = -2.0, JNT2_MAX = 2.0, JNT3_MIN = -2.0, JNT3_MAX = 2.0,
		JNT4_MIN = -2.0, JNT4_MAX = 2.0, JNT5_MIN = -2.0, JNT5_MAX = 2.0;
std::vector<double> voltages(6);

void forceCb(const geometry_msgs::WrenchStamped& wrench) {
	voltages[0] = wrench.wrench.force.x * JNT0_V_PER_N;
	voltages[1] = wrench.wrench.force.y * JNT1_V_PER_N;
	voltages[2] = wrench.wrench.force.z * JNT2_V_PER_N;
	voltages[3] = wrench.wrench.torque.x * JNT3_V_PER_NM;
	voltages[4] = wrench.wrench.torque.y * JNT4_V_PER_NM;
	voltages[5] = wrench.wrench.torque.z * JNT5_V_PER_NM;
}

int main (int argc, char** argv) {
	ros::init(argc,argv,"force_reflection_test");
	ros::NodeHandle nh;
	ros::Subscriber force_sub = nh.subscribe("/test_force_torque_sensor",1,forceCb);
	if(!nh.getParam("/joint0/v_per_n", JNT0_V_PER_N)) {
		ROS_INFO("Setting default value for joint0 voltage per newton: %f", JNT0_V_PER_N);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT0_V_PER_N);}
	if(!nh.getParam("/joint1/v_per_n", JNT1_V_PER_N)) {
		ROS_INFO("Setting default value for joint1 voltage per newton: %f", JNT1_V_PER_N);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT1_V_PER_N);}
	if(!nh.getParam("/joint2/v_per_n", JNT2_V_PER_N)) {
		ROS_INFO("Setting default value for joint2 voltage per newton: %f", JNT2_V_PER_N);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT2_V_PER_N);}
	if(!nh.getParam("/joint3/v_per_n", JNT3_V_PER_NM)) {
		ROS_INFO("Setting default value for joint3 voltage per newton: %f", JNT3_V_PER_NM);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT3_V_PER_NM);}
	if(!nh.getParam("/joint4/v_per_n", JNT4_V_PER_NM)) {
		ROS_INFO("Setting default value for joint4 voltage per newton: %f", JNT4_V_PER_NM);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT4_V_PER_NM);}
	if(!nh.getParam("/joint5/v_per_n", JNT5_V_PER_NM)) {
		ROS_INFO("Setting default value for joint5 voltage per newton: %f", JNT5_V_PER_NM);
	}
	else { ROS_INFO("Loaded value from param server: %f volts/newton", JNT5_V_PER_NM);}
	if(!nh.getParam("/rate", RATE)) {
		ROS_INFO("default rate set at %f Hz", RATE);
	}
	else {ROS_INFO("Rate set from param sever %f", RATE);}
	

	PhidgetVoltageOutputHandle Vout0_handle_, Vout1_handle_, Vout2_handle_;
	
	PhidgetReturnCode prc; //Dont need this, todo.
	prc = PhidgetVoltageOutput_create(&Vout0_handle_);
	prc = PhidgetVoltageOutput_create(&Vout1_handle_);
	prc = PhidgetVoltageOutput_create(&Vout2_handle_);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)Vout0_handle_, 525557);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)Vout1_handle_, 525557);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)Vout2_handle_, 525557);
	prc = Phidget_setChannel((PhidgetHandle)Vout0_handle_,0);
	prc = Phidget_setChannel((PhidgetHandle)Vout1_handle_,1);
	prc = Phidget_setChannel((PhidgetHandle)Vout2_handle_,2);
	prc = Phidget_openWaitForAttachment((PhidgetHandle)Vout0_handle_, 10000);
	std::cout<<"Return code on attachment motor 1: "<<prc<<std::endl;
	prc = Phidget_openWaitForAttachment((PhidgetHandle)Vout1_handle_, 10000);
	std::cout<<"Return code on attachment motor 2:"<<prc<<std::endl;
	prc = Phidget_openWaitForAttachment((PhidgetHandle)Vout2_handle_, 10000);
	std::cout<<"Return code on attachment motor 3: "<<prc<<std::endl;

	//ALL OF THIS NEEDS TO BE IN A CONSTRUCTOR IN A CLASS. TODO


	ros::Rate naptime(RATE);
	while(ros::ok()) {
		ros::spinOnce();
		prc = PhidgetVoltageOutput_setVoltage(Vout0_handle_,voltages[0]);
		//ROS_INFO("Setting voltage of %f at J_0", voltages[0]);
		prc = PhidgetVoltageOutput_setVoltage(Vout1_handle_,voltages[1]);
		prc = PhidgetVoltageOutput_setVoltage(Vout2_handle_,voltages[2]);
		naptime.sleep();
	}
	//DESTRUCTOR STUFF. TODO
	prc = Phidget_close((PhidgetHandle)Vout0_handle_);
	prc = Phidget_close((PhidgetHandle)Vout1_handle_);
	prc = Phidget_close((PhidgetHandle)Vout2_handle_);
	prc = PhidgetVoltageOutput_delete(&Vout0_handle_);
	
	prc = PhidgetVoltageOutput_delete(&Vout1_handle_);
	prc = PhidgetVoltageOutput_delete(&Vout2_handle_);
	return 0;
}