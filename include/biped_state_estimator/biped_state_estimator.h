#ifndef THOR_MANG_STATE_ESTIMATOR_H
#define THOR_MANG_STATE_ESTIMATOR_H

#include <Eigen/Dense>
#include <ros/node_handle.h>
#include <robot_transforms/robot_transforms.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Imu.h>

#include <cmath>

namespace Eigen {
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

typedef struct IMU
{
		IMU() : roll(0), pitch(0), yaw(0){}

		double roll;
		double pitch;
		double yaw;

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

typedef struct Twist
{
    Twist() : linear(0.0,0.0,0.0), angular(0.0,0.0,0.0) {}
    Twist(Eigen::Vector3d linear_, Eigen::Vector3d angular_) {
      linear = linear_;
      angular = angular_;
    }

    Eigen::Vector3d linear;
    Eigen::Vector3d angular;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} Twist;

namespace robot_tools {
class StateEstimator {
public:
	// Init
	StateEstimator();
	bool init(ros::NodeHandle nh, bool reset_on_start=false);
	void reset();

	// Input
	void setFeetForceZ(const double& left_z, const double& right_z);
	/**
	 * @brief setIMU
	 * @param orientation Orientation as quaternion(x,y,z,w)
	 */
	//void setIMU(double (&orientation)[4], double (&angular_velocity)[3], double (&linear_acceleration)[3]);
	void setIMU(double roll, double pitch, double yaw);
  void setIMU(const sensor_msgs::ImuConstPtr& imu_ptr);

	void setRobotTransforms(boost::shared_ptr<robot_tools::RobotTransforms> transforms_ptr);

	// Output
	Pose getPelvisPose();
	Pose getCOMinFootFrame();
	Eigen::Vector6d getCOMinFootFrameRPY();
	std::string getSupportFoot();

  // Getters
  bool isInitialized() const;

	// Update
	void update();
	void update(ros::Time current_time);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	bool checkSupportFootChange();
	void setSupportFoot(std::string foot_name);
	double getFootHeight(std::string foot_name);
	std::string otherFoot(std::string foot_name);

  void publishPelvisWorldPose(ros::Time current_time);
  void publishPelvisOdometry(ros::Time current_time);
	void publishGroundPoint(ros::Time current_time);
  void publishCOM(ros::Time current_time);
  void publishPose(const Pose& pose, const ros::Publisher& pub, ros::Time current_time, std::string frame = "world", bool add_to_array=false);
  void publishOdometry(const Pose& pose, const Twist& twist, const ros::Publisher& pub, ros::Time current_time, std::string frame = "world");
	void sysCommandCb(const std_msgs::StringConstPtr& msg);

	Eigen::Quaterniond imuToRot(IMU imu) const;
	Eigen::Quaterniond rpyToRot(double roll, double pitch, double yaw) const;
	Eigen::Vector3d rotToRpy(Eigen::Matrix3d rot) const;

  ros::Publisher pelvis_pose_pub_;
  ros::Publisher pelvis_odom_pub_;
	ros::Publisher ground_point_pub_;
	ros::Publisher com_pub_;
  ros::Publisher ground_com_pub_;
  ros::Publisher footstep_vis_pub_;
  ros::Publisher imu_orientation_pub_;
	ros::Subscriber syscmd_sub_;

	bool initialized_;
	bool resetted_;

	ros::NodeHandle nh_;				// Node handle in which the state estimator is running
	Pose world_pose_;						// Pose of the robot pelvis relative to world frame
	double yaw_;
	std::string support_foot_;	// Foot on groud: 'right_foot' or 'left_foot'
	double height_treshold_;		// The non-supporting foot has to pass this limit before it can be a supporting foot again
	double ankle_z_offset_;
	Pose ground_point_;					// Contact point of the robot with the world. Moves with each step.

  Eigen::Vector3d com_offset_;

	bool height_treshold_passed_;				// True if the non-supporting foot was higher than the threshold relative to the supporting foot

	// Parameters
	std::string pelvis_name_;			// Name of the pelvis
  std::string tf_odom_frame_;   // Name of tf frame
  std::string imu_world_frame_;
  Eigen::Quaterniond imu_world_rotation_offset_;
	std::string right_foot_name_; // Name of the right foot
	std::string left_foot_name_;	// Name of the left foot

	// Input
	double left_force_;																			// Measured force of feet ft sensors in z direction
	double right_force_;
	boost::shared_ptr<robot_tools::RobotTransforms> robot_transforms_ptr_; // Calculates forward kinematics
	IMU imu_;																								// IMU data

  unsigned int id_;
};
}

#endif
