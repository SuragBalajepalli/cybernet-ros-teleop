#include <libphidget22/phidget22.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cstdlib>


int main(int argc, char** argv) {
	
	ros::init(argc,argv,"Vout_test");
	ros::NodeHandle nh;
	
	ros::Publisher pub = nh.advertise<std_msgs::Float32>("curr_vol",1);	
	PhidgetVoltageOutputHandle Vout_handle_;
	
	PhidgetReturnCode prc;
	prc = PhidgetVoltageOutput_create(&Vout_handle_);
	std::cout<<"Return code on creation"<<prc<<std::endl;
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)Vout_handle_, 525557);
	prc = Phidget_openWaitForAttachment((PhidgetHandle)Vout_handle_, 5000);
	std::cout<<"Return code on attachment"<<prc<<std::endl;
	std_msgs::Float32 val;
	

	

	
	double voltage = 0.0;
	double minVoltage = 0.0;
	double maxVoltage = 0.0;
	float offset = 0.1;
	prc = PhidgetVoltageOutput_getMinVoltage(Vout_handle_, &minVoltage);
	prc = PhidgetVoltageOutput_getMaxVoltage(Vout_handle_, &maxVoltage);

	std::cout<<"min voltage is"<<minVoltage<<std::endl;
	std::cout<<"max voltage is"<<maxVoltage<<std::endl;
	ros::Rate naptime(1.0);

	while(ros::ok()) {
		prc = PhidgetVoltageOutput_setVoltage(Vout_handle_,voltage);
		voltage += offset;
		std::cout<<"offset "<<offset<<std::endl;
		if(offset >= maxVoltage || offset<=minVoltage) offset*= -1;
		val.data = voltage;
		pub.publish(val);
		naptime.sleep();
		
	}
	prc = Phidget_close((PhidgetHandle)Vout_handle_);
	std::cout<<"error while closing"<<prc<<std::endl;
	prc = PhidgetVoltageOutput_delete(&Vout_handle_);
	std::cout<<"error while deleting"<<prc<<std::endl;
	return 0;
}