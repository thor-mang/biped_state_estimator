#include <biped_state_estimator/biped_state_estimator.h>

namespace Thor {
// Init
StateEstimator::StateEstimator() {

}

bool StateEstimator::init(ros::NodeHandle &nh) {
	// Load params
	nh_ = nh;
	nh.param("pelvis_name", pelvis_name_, std::string("pelvis"));
	nh.param("right_foot_name", right_foot_name_, std::string("r_foot"));
	nh.param("left_foot_name", left_foot_name_, std::string("l_foot"));
	nh.param("height_treshold", height_treshold_, 0.05);

	// Init state estimation
	support_foot_ = "undefined"; // Starting in double support
	world_orientation_ = Eigen::Quaterniond::Identity();
	world_position_ = Eigen::Vector3d::Zero();
	ground_point_ = Eigen::Vector3d::Zero();	// Init when first leg leaves ground.

	height_treshold_passed_ = false;

	left_force_ = 0;
	right_force_ = 0;
	imu_ = IMU();

	pelvis_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pelvis_pose", 1000);

	ROS_INFO("State estimation initialized.");
	return true;
}

// Input
void StateEstimator::setFeetForceZ(const double& left_z, const double& right_z) {
	left_force_ = left_z;
	right_force_ = right_z;
}

void StateEstimator::setIMU(double (&orientation)[4], double (&angular_velocity)[3], double (&linear_acceleration)[3]) {
	imu_.orientation = Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]);
	imu_.angular_velocity = Eigen::Vector3d(angular_velocity[0], angular_velocity[1], angular_velocity[2]);
	imu_.linear_acceleration = Eigen::Vector3d(linear_acceleration[0], linear_acceleration[1], linear_acceleration[2]);
}

void StateEstimator::setRobotTransforms(robot_tools::RobotTransforms& transforms) {
	robot_transforms_ptr_.reset(&transforms);
}

// Output
Pose StateEstimator::getPelvisPose() {
	return Pose(world_orientation_, world_position_);
}

std::string StateEstimator::getSupportFoot() {
	return support_foot_;
}

// Update
void StateEstimator::update() {
	double left_height = getFootHeight(left_foot_name_);
	double right_height = getFootHeight(right_foot_name_);

	double difference = left_height - right_height;

	std::string lower_foot = difference < 0 ? "left_foot" : "right_foot";
	if (std::abs(difference) > height_treshold_ && !height_treshold_passed_) {
		height_treshold_passed_ = true;
		ROS_INFO_STREAM("Height threshold passed.");
	}
	if (lower_foot != support_foot_ && height_treshold_passed_) {
		height_treshold_passed_ = false;
		setSupportFoot(lower_foot);
	}

	world_orientation_ = imu_.orientation;
	Eigen::Vector3d support_foot_to_pelvis = -robot_transforms_ptr_->getTransform(support_foot_).translation();
	world_position_ = ground_point_ + support_foot_to_pelvis;
}


// Private
Eigen::Vector3d StateEstimator::getLeftToRightDistance() {
	Eigen::Vector3d left_pos = robot_transforms_ptr_->getTransform(left_foot_name_).translation();
	Eigen::Vector3d right_pos = robot_transforms_ptr_->getTransform(right_foot_name_).translation();

	return right_pos - left_pos;
}

void StateEstimator::setSupportFoot(std::string foot_name) {
	ROS_INFO_STREAM("Changing support foot to: " << foot_name);
	if (support_foot_ == "undefined") {
		// init ground position to supporting foot
		ground_point_ = imu_.orientation * robot_transforms_ptr_->getTransform(foot_name).translation();
		ground_point_.z() = 0;
		ROS_INFO_STREAM("Initializing ground_point to: " << ground_point_.x() << ", " << ground_point_.y() << ", " << ground_point_.z());
	} else {
		// add distance between feet to ground position
		Eigen::Vector3d left_to_right = getLeftToRightDistance();
		double sign = foot_name == right_foot_name_ ? 1.0 : -1.0;
		ground_point_ += sign * left_to_right;
		ROS_INFO_STREAM("Moving ground_point to: " << ground_point_.x() << ", " << ground_point_.y() << ", " << ground_point_.z());
	}

	support_foot_ = foot_name;
}


double StateEstimator::getFootHeight(std::string foot_name) {
	Eigen::Affine3d pose	= robot_transforms_ptr_->getTransform(foot_name);
	Eigen::Vector3d position = pose.translation();
	Eigen::Vector3d world_position = imu_.orientation * position;
	return -world_position.z();
}

void StateEstimator::publishPelvisWorldPose() {
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "world";
	pose.header.stamp = ros::Time::now();

	pose.pose.orientation.x = world_orientation_.x();
	pose.pose.orientation.y = world_orientation_.y();
	pose.pose.orientation.z = world_orientation_.z();
	pose.pose.orientation.w = world_orientation_.w();

	pose.pose.position.x = world_position_.x();
	pose.pose.position.y = world_position_.y();
	pose.pose.position.z = world_position_.z();

	pelvis_pose_pub_.publish(pose);
}
} // namespace Thor
