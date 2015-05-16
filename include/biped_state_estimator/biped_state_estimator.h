#ifndef THOR_MANG_STATE_ESTIMATOR_H
#define THOR_MANG_STATE_ESTIMATOR_H

#include <Eigen/Dense>
#include <ros/node_handle.h>
#include <robot_transforms/robot_transforms.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>


#include <cmath>


typedef struct IMU
{
		IMU() : orientation(1.0,0.0,0.0,0.0),angular_velocity(0.0,0.0,0.0),linear_acceleration(0.0,0.0,0.0){}

		Eigen::Quaterniond orientation;
		Eigen::Vector3d angular_velocity;
		Eigen::Vector3d linear_acceleration;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} IMU;

typedef struct Pose
{
		Pose() : orientation(1.0,0.0,0.0,0.0),position(0.0,0.0,0.0) {}
		Pose(Eigen::Quaterniond orientation_, Eigen::Vector3d position_) {
			orientation = orientation_;
			position = position_;
		}

		Eigen::Quaterniond orientation;
		Eigen::Vector3d position;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} Pose;

namespace robot_tools {
class StateEstimator {
public:
	// Init
	StateEstimator();
	bool init(ros::NodeHandle nh);

	// Input
	void setFeetForceZ(const double& left_z, const double& right_z);
	void setIMU(double (&orientation)[4], double (&angular_velocity)[3], double (&linear_acceleration)[3]);
	void setRobotTransforms(robot_tools::RobotTransforms& transforms);

	// Output
	Pose getPelvisPose();
	std::string getSupportFoot();

	// Update
	void update();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	void reset();
	void setSupportFoot(std::string foot_name);
	Eigen::Vector3d getLeftToRightDistance();
	double getFootHeight(std::string foot_name);

	void publishPelvisWorldPose();
	void sysCommandCb(const std_msgs::StringConstPtr& msg);

	ros::Publisher pelvis_pose_pub_;
	ros::Subscriber syscmd_sub_;

	bool initialized_;

	ros::NodeHandle nh_;								// Node handle in which the state estimator is running
	Eigen::Quaterniond world_orientation_; // Orientation of the robot pelvis relative to world frame
	Eigen::Vector3d world_position_;    // Position of the robot pelvis relative to world frame
	std::string support_foot_;					// Foot on groud: 'right_foot' or 'left_foot'
	double height_treshold_;						// The non-supporting foot has to pass this limit before it can be a supporting foot again
	Eigen::Vector3d ground_point_;		  // Contact point of the robot with the world. Moves with each step.

	bool height_treshold_passed_;				// True if the non-supporting foot was higher than the threshold relative to the supporting foot

	// Parameters
	std::string pelvis_name_;			// Name of the pelvis
	std::string right_foot_name_; // Name of the right foot
	std::string left_foot_name_;	// Name of the left foot

	// Input
	double left_force_;																			// Measured force of feet ft sensors in z direction
	double right_force_;
	boost::shared_ptr<robot_tools::RobotTransforms> robot_transforms_ptr_; // Calculates forward kinematics
	IMU imu_;																								// IMU data
};
}

#endif
