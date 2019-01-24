//Virt attractor control from joint 0,1,2 of cybernet hand device. Need to set params

#include <libphidget22/phidget22.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cstdlib>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <test_pkg/virt_attr_service.h>

geometry_msgs::PoseStamped virt_attr_pose;
sensor_msgs::JointState prev_enc_vals;
bool reset_mode = false, init = true; //For future use of moving the hand controller without changing vals
double RATE = 100, JNT0_MM_PER_TICK = 1, JNT1_MM_PER_TICK = 1, JNT2_MM_PER_TICK = 1;

bool setZeroCb(test_pkg::virt_attr_serviceRequest& request, test_pkg::virt_attr_serviceResponse& response) {
	ROS_INFO("Command to set to zero recieved");
	virt_attr_pose.pose.position.x = 0.0;
	virt_attr_pose.pose.position.y = 0.0;
	virt_attr_pose.pose.position.z = 0.0;
	response.success = true;
	return true;
}


void encoderCb(const sensor_msgs::JointState& encoder_vals) {
	//On startup, it ensures that the virtual attractor is at origin. 
	if(init) { //borderline spaghetti code; find another way to do this
		prev_enc_vals.position.resize(3);
		prev_enc_vals.position[0] = encoder_vals.position[0];
		prev_enc_vals.position[1] = encoder_vals.position[1];
		prev_enc_vals.position[2] = encoder_vals.position[2];
		init = false;
	}
	else {
		if(!reset_mode) { //In "reset mode", any movement in the joystick does not reflect on the end effector
			virt_attr_pose.pose.position.x += (encoder_vals.position[0] - prev_enc_vals.position[0]) * JNT0_MM_PER_TICK;
			virt_attr_pose.pose.position.y += (encoder_vals.position[1] - prev_enc_vals.position[1]) * JNT1_MM_PER_TICK;
			virt_attr_pose.pose.position.z += (encoder_vals.position[2] - prev_enc_vals.position[2]) * JNT2_MM_PER_TICK;
			}
		prev_enc_vals.position[0] = encoder_vals.position[0];
		prev_enc_vals.position[1] = encoder_vals.position[1];
		prev_enc_vals.position[2] = encoder_vals.position[2];
	}
}

int main(int argc, char** argv) {
	ros::init(argc,argv,"Virt_attr_test_3DOF");
	ros::NodeHandle nh;
	ros::Publisher virt_attr_pub = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_3dof",1);
	ros::Subscriber encoder_sub = nh.subscribe("/joint_states",1, encoderCb); //Will remap topic in the future, fix this then
	ros::ServiceServer zero_service = nh.advertiseService("set_v_attr_zero", setZeroCb);
	ros::Rate naptime(RATE);

	if(!nh.getParam("/joint0/mm_per_tick", JNT0_MM_PER_TICK)) {
		ROS_INFO("Loading default value for joint_0 mm per tick %f", JNT0_MM_PER_TICK);
	}
	if(!nh.getParam("/joint1/mm_per_tick", JNT1_MM_PER_TICK)) {
		ROS_INFO("Loading default value for joint_1 mm per tick %f", JNT1_MM_PER_TICK);
	}
	if(!nh.getParam("/joint2/mm_per_tick", JNT2_MM_PER_TICK)) {
		ROS_INFO("Loading default value for joint_2 mm per tick %f", JNT2_MM_PER_TICK);
	}
	if(!nh.getParam("/rate", RATE)) {
		ROS_INFO("default rate set at %f Hz", RATE);
	}
	while(ros::ok()) {
		ros::spinOnce();
		virt_attr_pub.publish(virt_attr_pose);
		naptime.sleep();
	}
}