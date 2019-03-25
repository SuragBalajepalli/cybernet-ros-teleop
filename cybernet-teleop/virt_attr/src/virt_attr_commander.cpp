//Virt attractor control from joint 0,1,2 of cybernet hand device. 
//To do, angular stuff
#include <libphidget22/phidget22.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <virt_attr/virt_attr_zero.h>
#include <virt_attr/toggle_reset.h>


geometry_msgs::PoseStamped virt_attr_pose;
Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();

sensor_msgs::JointState prev_enc_vals;
bool reset_mode = false, init = true; //For future use of moving the hand controller without changing vals
double RATE = 1000, JNT0_M_PER_TICK = 0.0000762/2, JNT1_M_PER_TICK = 0.0000762/2, JNT2_M_PER_TICK = 0.0000762/2; // default in case param server is empty
double JNT4_RAD_PER_TICK = 0.00019, JNT5_RAD_PER_TICK = 0.00019, JNT6_RAD_PER_TICK = 0.00019; 
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
		virt_attr_pose.pose.position.x += (encoder_vals.position[0] - prev_enc_vals.position[0]) * JNT0_M_PER_TICK;
		virt_attr_pose.pose.position.y += (encoder_vals.position[1] - prev_enc_vals.position[1]) * JNT1_M_PER_TICK;
		virt_attr_pose.pose.position.z += (encoder_vals.position[2] - prev_enc_vals.position[2]) * JNT2_M_PER_TICK;
		}
	if(prev_enc_vals.position.size() != 6) prev_enc_vals.position.resize(6);
	prev_enc_vals.position[0] = encoder_vals.position[0];
	prev_enc_vals.position[1] = encoder_vals.position[1];
	prev_enc_vals.position[2] = encoder_vals.position[2];
	
	init = false; //Do not need this to be repeatedly set to false but no harm, really.
}

void angular_encoderCb(const sensor_msgs::JointState& encoder_vals_ang) {
	//for orientation, consider a ZYX Euler representation. 
	// R = Rz(m4) * Ry(m5) * Rx(m6)
	//On startup, it ensures that the virtual attractor is at origin.
	if(init) {
		prev_enc_vals.position.resize(6);

		prev_enc_vals.position[3] = encoder_vals_ang.position[0];
		prev_enc_vals.position[4] = encoder_vals_ang.position[1];
		prev_enc_vals.position[5] = encoder_vals_ang.position[2];
		init = false;
	}
	else {
		//construct rotation matrix from change in angles
		//assuming a start from 0
			double theta_z = (encoder_vals_ang.position[0] - prev_enc_vals.position[3]) * JNT4_RAD_PER_TICK;
			double theta_y = (encoder_vals_ang.position[1] - prev_enc_vals.position[4]) * -JNT5_RAD_PER_TICK;
			double theta_x = (encoder_vals_ang.position[2] - prev_enc_vals.position[5]) * JNT6_RAD_PER_TICK;
			Eigen::Matrix3d rot_mat_change, rot_z, rot_y, rot_x;
			
			rot_z<<cos(theta_z), -sin(theta_z),0,
					sin(theta_z), cos(theta_z),0,
					0,0,1;

			rot_y<<cos(theta_y),0,sin(theta_y),
					0,1,0,
					-sin(theta_y),0,cos(theta_y);

			rot_x<<1,0,0,
					0,cos(theta_x),-sin(theta_x),
					0,sin(theta_x),cos(theta_x);

			//this is rotation matrix from t-1 to t
			rot_mat_change = rot_z * rot_y;
			

			//convert rot mat into quaternion form
			rot_mat = rot_mat * rot_mat_change;
			//Eigen's quaternion conversion is fast
			Eigen::Quaterniond quat(rot_mat);
			quat.normalize();
			virt_attr_pose.pose.orientation.w = quat.w();
			virt_attr_pose.pose.orientation.x = quat.x();
			virt_attr_pose.pose.orientation.y = quat.y();
			virt_attr_pose.pose.orientation.z = quat.z();
	}
	prev_enc_vals.position[3] = encoder_vals_ang.position[0];
	prev_enc_vals.position[4] = encoder_vals_ang.position[1];
	prev_enc_vals.position[5] = encoder_vals_ang.position[2];


}


int main(int argc, char** argv) {
	ros::init(argc,argv,"Virt_attr_commander");
	ros::NodeHandle nh;
	ros::Publisher virt_attr_pub = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);
	ros::Subscriber linear_encoder_sub = nh.subscribe("/joint_states_linear",1, linear_encoderCb); //Will remap topic in the future, fix this then
	ros::Subscriber angular_encoder_sub = nh.subscribe("/joint_states_angular",1, angular_encoderCb);
	ros::ServiceServer zero_service = nh.advertiseService("set_v_attr_zero", setZeroCb);
	ros::ServiceServer reset_service = nh.advertiseService("toggle_reset_mode", resetToggleCb);
	virt_attr_pose.header.frame_id = "map";

	if(!nh.getParam("/joint0/m_per_tick", JNT0_M_PER_TICK)) {
		ROS_INFO("Loading default value for joint_0 m per tick %f", JNT0_M_PER_TICK);
	}
	else { ROS_INFO("Loaded value from param server: %f m/tick", JNT1_M_PER_TICK);}
	if(!nh.getParam("/joint1/m_per_tick", JNT1_M_PER_TICK)) {
		ROS_INFO("Loading default value for joint_1 m per tick %f", JNT1_M_PER_TICK);
	}
	else { ROS_INFO("Loaded value from param server: %f m/tick", JNT1_M_PER_TICK);}
	if(!nh.getParam("/joint2/m_per_tick", JNT2_M_PER_TICK)) {
		ROS_INFO("Loading default value for joint_2 m per tick %f", JNT2_M_PER_TICK);
	}
	else { ROS_INFO("Loaded value from param server: %f m/tick", JNT2_M_PER_TICK);}
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