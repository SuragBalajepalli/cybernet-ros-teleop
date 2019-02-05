#include <ros/ros.h>
#include <libphidget22/phidget22.h>
#include <cstdlib>
#include <sensor_msgs/JointState.h>
//subscribe to encoder
//check encoder delta. Keep increasing voltage until delta = 0
double encoder_t_minus_1 = 0;
double delta_enc;
int curr_motor = 2;
double RATE = 10.0;
void encoderCb(const sensor_msgs::JointState& encoder_vals) {
	delta_enc = encoder_vals.position[curr_motor] - encoder_t_minus_1;
	ROS_INFO("Delta enc updated to %f", delta_enc);

}

int main(int argc, char** argv){
	ros::init(argc,argv,"deadband_measurement");
	ros::NodeHandle nh;
	ros::Subscriber enc_sub = nh.subscribe("/joint_states",1,encoderCb);
	double voltage_cmd =0.0;
	double voltage_step = -0.005;
	PhidgetVoltageOutputHandle Vout0_handle_;
	PhidgetReturnCode prc; //Dont need this, todo.
	prc = PhidgetVoltageOutput_create(&Vout0_handle_);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)Vout0_handle_, 525557);
	prc = Phidget_setChannel((PhidgetHandle)Vout0_handle_,curr_motor);
	
	prc = Phidget_openWaitForAttachment((PhidgetHandle)Vout0_handle_, 5000);
	std::cout<<"Return code on attachment"<<prc<<std::endl;
	ros::Rate naptime(RATE);
	if(prc != 0) {
		ROS_INFO("cant connect");
		return -1;
	}

	while(ros::ok()) {
		ros::spinOnce();
		if(abs(delta_enc) < 2) { //maybe encoders are noisy, should be a threshold?
			ROS_INFO("No motion, at %f", voltage_cmd);
			voltage_cmd += voltage_step;
		}
		else { //robot motion detected, need to go in the negative direction- how to?
			ROS_INFO("Robot moved at %f", voltage_cmd);
			
		}
		prc = PhidgetVoltageOutput_setVoltage(Vout0_handle_,voltage_cmd);
		naptime.sleep();
		ros::spinOnce();
	}
	prc = Phidget_close((PhidgetHandle)Vout0_handle_);
	prc = PhidgetVoltageOutput_delete(&Vout0_handle_);
	return 0;

}