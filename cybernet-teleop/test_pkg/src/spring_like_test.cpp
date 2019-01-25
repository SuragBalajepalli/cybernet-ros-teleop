#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

double K = 0.01;
double Kz = -0.02;
ros::Publisher pub;
void Cb(const geometry_msgs::PoseStamped& pos) {
	geometry_msgs::WrenchStamped des_force;
	des_force.wrench.force.x = pos.pose.position.x * K;
	des_force.wrench.force.y = pos.pose.position.y * K;
	des_force.wrench.force.z = pos.pose.position.z * Kz;
	ROS_INFO_STREAM("Setting force"<<des_force);
	pub.publish(des_force);
}

int main(int argc, char** argv) {
	ros::init(argc,argv,"spring_test");
	ros::NodeHandle nh;
	pub = nh.advertise<geometry_msgs::WrenchStamped>("/test_force_torque_sensor",1);
	ros::Subscriber sub = nh.subscribe("/Virt_attr_pose",1,Cb);
	while(ros::ok()) {
		ros::spin();
	}
}