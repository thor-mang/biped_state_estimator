#include <biped_state_estimator/biped_state_estimator.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#include <kdl/frames.hpp>

boost::shared_ptr<robot_tools::RobotTransforms> transforms_ptr;
boost::shared_ptr<robot_tools::StateEstimator> estimator_ptr;
ros::Subscriber joint_state_sub;
ros::Subscriber imu_sub;
bool got_imu = false;
bool got_joint_states = false;

void joint_state_cb(const sensor_msgs::JointStateConstPtr& joint_state_ptr) {
	for (unsigned int i = 0; i < joint_state_ptr->name.size(); i++) {
		transforms_ptr->updateState(joint_state_ptr->name[i], joint_state_ptr->position[i]);
	}
	estimator_ptr->update(joint_state_ptr->header.stamp);
	got_joint_states = true;
}

void imu_cb(const sensor_msgs::ImuConstPtr& imu_ptr) {
	KDL::Rotation rot = KDL::Rotation::Quaternion(imu_ptr->orientation.x, imu_ptr->orientation.y, imu_ptr->orientation.z, imu_ptr->orientation.w);
	double roll, pitch, yaw;
	rot.GetRPY(roll, pitch, yaw);
	// ROS_INFO_STREAM("imu rpy: " << roll << ", " << pitch << ", " << yaw);
	double orientation[4];
	KDL::Rotation no_yaw = KDL::Rotation::RPY(roll, pitch, 0);
	no_yaw.GetQuaternion(orientation[0], orientation[1], orientation[2], orientation[3]);
//	Eigen::Quaterniond rot(imu_ptr->orientation.w, imu_ptr->orientation.x, imu_ptr->orientation.y, imu_ptr->orientation.z);
//	Eigen::Matrix3d rot_mat(rot);
//	Eigen::Vector3d rpy = rot_mat.eulerAngles(0, 1, 2);
//	Eigen::Quaterniond rot_no_yaw = Eigen::AngleAxisd(rpy(0),		Eigen::Vector3d::UnitX())
//																* Eigen::AngleAxisd(rpy(1),		Eigen::Vector3d::UnitY())
//																* Eigen::AngleAxisd(0.0,			Eigen::Vector3d::UnitZ());
//	double orientation[4];
//	orientation[0] = rot_no_yaw.x();
//	orientation[1] = rot_no_yaw.y();
//	orientation[2] = rot_no_yaw.z();
//	orientation[3] = rot_no_yaw.w();

	double linear_acc[3], angular_velocity[3];
	linear_acc[0] = imu_ptr->linear_acceleration.x;
	linear_acc[1] = imu_ptr->linear_acceleration.y;
	linear_acc[2] = imu_ptr->linear_acceleration.z;

	angular_velocity[0] = imu_ptr->angular_velocity.x;
	angular_velocity[1] = imu_ptr->angular_velocity.y;
	angular_velocity[2] = imu_ptr->angular_velocity.z;
//	estimator_ptr->setIMU(orientation, angular_velocity, linear_acc);
	got_imu = true;
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "state_estimator_test_node");

	ros::NodeHandle nh("state_estimator");

	joint_state_sub = nh.subscribe("/joint_states", 1000, &joint_state_cb);
	imu_sub = nh.subscribe("/thor_mang/pelvis_imu", 1000, &imu_cb);

	transforms_ptr.reset(new robot_tools::RobotTransforms);
	transforms_ptr->init();

	estimator_ptr.reset(new robot_tools::StateEstimator);
	estimator_ptr->setRobotTransforms(transforms_ptr);
	while (!(got_imu && got_joint_states)) {
		usleep(8000);
		ros::spinOnce();
	}
	estimator_ptr->init(nh, true);

	ros::spin();
} 
