//Virt attractor control from joint 0,1,2 of cybernet hand device. 
//To do, angular stuff
#include <libphidget22/phidget22.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cstdlib>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <virt_attr/virt_attr_zero.h>
#include <virt_attr/toggle_reset.h>


geometry_msgs::PoseStamped virt_attr_pose;
sensor_msgs::JointState prev_enc_vals;
bool reset_mode = false, init = true; //For future use of moving the hand controller without changing vals
double RATE = 100, JNT0_MM_PER_TICK = 0.00762, JNT1_MM_PER_TICK = 0.00762, JNT2_MM_PER_TICK = 0.00762; // default in case param server is empty

bool setZeroCb(virt_attr::virt_attr_zeroRequest& request, virt_attr::virt_attr_zeroResponse& response) {
	//A call to this service sets the virtual attractor pose to origin
	ROS_INFO("Command to set to zero recieved");
	virt_attr_pose.pose.position.x = 0.0;
	virt_attr_pose.pose.position.y = 0.0;
	virt_attr_pose.pose.position.z = 0.0;
	response.success = true;
	return true;
}

bool resetToggleCb(virt_attr::toggle_resetRequest& request, virt_attr::toggle_resetResponse& response) {
	//A call to this service toggles between reset mode and normal mode. 
	//By default, the system is in normal mode
	//Reset mode is analogous to lifting your mouse off the table and moving it
	//Used when hand controller's mechanical limits are reached but motion is still reqd
	reset_mode = !reset_mode;
	ROS_INFO("Reset mode has been toggled");
	if(reset_mode) {
		ROS_INFO("Currently in reset mode");
		response.status = "Reset mode";
		
	}
	else {
		ROS_INFO("Currently in normal mode");
		response.status = "Normal mode";
	}

	return true;
}


void linear_encoderCb(const sensor_msgs::JointState& encoder_vals) {
	//On startup, it ensures that the virtual attractor is at origin. 
	//borderline spaghetti code; find another way to do this
	if(!reset_mode && !init) { //In "reset mode", any movement in the joystick does not reflect on the end effector
		virt_attr_pose.pose.position.x += (encoder_vals.position[0] - prev_enc_vals.position[0]) * JNT0_MM_PER_TICK;
		virt_attr_pose.pose.position.y += (encoder_vals.position[1] - prev_enc_vals.position[1]) * JNT1_MM_PER_TICK;
		virt_attr_pose.pose.position.z += (encoder_vals.position[2] - prev_enc_vals.position[2]) * JNT2_MM_PER_TICK;
		}
	if(prev_enc_vals.position.size() != 6) prev_enc_vals.position.resize(6);
	prev_enc_vals.position[0] = encoder_vals.position[0];
	prev_enc_vals.position[1] = encoder_vals.position[1];
	prev_enc_vals.position[2] = encoder_vals.position[2];
	prev_enc_vals.position[3] = encoder_vals.position[3];
	prev_enc_vals.position[4] = encoder_vals.position[4];
	prev_enc_vals.position[5] = encoder_vals.position[5];
	init = false; //Do not need this to be repeatedly set to false but no harm, really.
}

void angular_encoderCb(const sensor_msgs::JointState& encoder_vals) {
	//On startup, it ensures that the virtual attractor is at origin. 
	//borderline spaghetti code; find another way to do this
	if(!reset_mode && !init) { //In "reset mode", any movement in the joystick does not reflect on the end effector
		// this is wrong for angular, fix it
		virt_attr_pose.pose.position.x += (encoder_vals.position[3] - prev_enc_vals.position[3]) * JNT0_MM_PER_TICK;
		virt_attr_pose.pose.position.y += (encoder_vals.position[4] - prev_enc_vals.position[4]) * JNT1_MM_PER_TICK;
		virt_attr_pose.pose.position.z += (encoder_vals.position[5] - prev_enc_vals.position[5]) * JNT2_MM_PER_TICK;
		}
	if(prev_enc_vals.position.size() != 6) prev_enc_vals.position.resize(6);
	prev_enc_vals.position[3] = encoder_vals.position[3];
	prev_enc_vals.position[4] = encoder_vals.position[4];
	prev_enc_vals.position[5] = encoder_vals.position[5];
	init = false; //Do not need this to be repeatedly set to false but no harm, really.
}


int main(int argc, char** argv) {
	ros::init(argc,argv,"Virt_attr_commander");
	ros::NodeHandle nh;
	ros::Publisher virt_attr_pub = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);
	ros::Subscriber linear_encoder_sub = nh.subscribe("/joint_states_linear",1, linear_encoderCb); //Will remap topic in the future, fix this then
	ros::Subscriber angular_encoder_sub = nh.subscribe("/joint_states_angular",1, angular_encoderCb);
	ros::ServiceServer zero_service = nh.advertiseService("set_v_attr_zero", setZeroCb);
	ros::ServiceServer reset_service = nh.advertiseService("toggle_reset_mode", resetToggleCb);

	if(!nh.getParam("/joint0/mm_per_tick", JNT0_MM_PER_TICK)) {
		ROS_INFO("Loading default value for joint_0 mm per tick %f", JNT0_MM_PER_TICK);
	}
	else { ROS_INFO("Loaded value from param server: %f mm/tick", JNT0_MM_PER_TICK);}
	if(!nh.getParam("/joint1/mm_per_tick", JNT1_MM_PER_TICK)) {
		ROS_INFO("Loading default value for joint_1 mm per tick %f", JNT1_MM_PER_TICK);
	}
	else { ROS_INFO("Loaded value from param server: %f mm/tick", JNT1_MM_PER_TICK);}
	if(!nh.getParam("/joint2/mm_per_tick", JNT2_MM_PER_TICK)) {
		ROS_INFO("Loading default value for joint_2 mm per tick %f", JNT2_MM_PER_TICK);
	}
	else { ROS_INFO("Loaded value from param server: %f mm/tick", JNT2_MM_PER_TICK);}
	//THREE MORE PARAM LOADS
	if(!nh.getParam("/rate", RATE)) {
		ROS_INFO("default rate set at %f Hz", RATE);
	}
	else {ROS_INFO("Rate set from param sever %f", RATE);}
	

	ros::Rate naptime(RATE);
	while(ros::ok()) {
		ros::spinOnce();
		virt_attr_pub.publish(virt_attr_pose);
		naptime.sleep();
	}
}