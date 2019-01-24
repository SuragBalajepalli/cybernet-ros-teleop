#include "motor_controller/motor_controller.h"


MotorController::MotorController(ros::NodeHandle& nh) {
	Vout_handles.resize(6);	
	for(int itr = 0; itr < 6; itr++) PhidgetVoltageOutput_create(&Vout_handles[itr]);

	for(int itr = 0; itr < 3; itr++) Phidget_setDeviceSerialNumber((PhidgetHandle)Vout_handles[itr], device_1_sn);  
	for(int itr = 3; itr < 6; itr++) Phidget_setDeviceSerialNumber((PhidgetHandle)Vout_handles[itr], device_2_sn);

		ROS_INFO("HERE");
	prc = Phidget_setChannel((PhidgetHandle)Vout_handles[0],0);
	prc = Phidget_setChannel((PhidgetHandle)Vout_handles[1],1);
	prc = Phidget_setChannel((PhidgetHandle)Vout_handles[2],2);
	prc = Phidget_setChannel((PhidgetHandle)Vout_handles[3],0);
	prc = Phidget_setChannel((PhidgetHandle)Vout_handles[4],1);
	prc = Phidget_setChannel((PhidgetHandle)Vout_handles[5],2);

	for(int itr = 0; itr < 6; itr++) {
		prc = Phidget_openWaitForAttachment((PhidgetHandle)Vout_handles[itr], timeout);
		if(prc == 3) ROS_INFO("Motor %i timeout", itr);
	}
	
}

MotorController::~MotorController() {

		for(int itr = 0; itr < 6; itr++) Phidget_close((PhidgetHandle)Vout_handles[itr]);
		for(int itr = 0; itr < 6; itr++) PhidgetVoltageOutput_delete(&Vout_handles[itr]);

}

bool MotorController::setVoltage(int motor_index, double voltage) {
	prc = PhidgetVoltageOutput_setVoltage(Vout_handles[motor_index],voltage);
	if(!prc) ROS_INFO("Error setting voltage to motor: %i", motor_index);

}


