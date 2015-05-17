#include <biped_state_estimator/biped_state_estimator.h>

namespace robot_tools {
// Init
StateEstimator::StateEstimator():
	left_force_(0.0),
	right_force_(0.0),
	pelvis_name_(""),
	right_foot_name_(""),
	left_foot_name_(""),
	height_treshold_passed_(false),
	height_treshold_(0.0),
	initialized_(false)
{}

bool StateEstimator::init(ros::NodeHandle nh) {
	// Load params
	nh_ = nh;
	nh.param("pelvis_name", pelvis_name_, std::string("pelvis"));
	nh.param("right_foot_name", right_foot_name_, std::string("r_foot"));
	nh.param("left_foot_name", left_foot_name_, std::string("l_foot"));
	nh.param("height_treshold", height_treshold_, 0.05);

	// Init state estimation
	support_foot_ = "undefined"; // Starting in double support
	world_pose_ = Pose();
	ground_point_.orientation = imu_.orientation * robot_transforms_ptr_->getTransform(left_foot_name_).linear();
	ground_point_.position = imu_.orientation * robot_transforms_ptr_->getTransform(left_foot_name_).translation();
	ground_point_.position.z() = 0;

	height_treshold_passed_ = false;

	left_force_ = 0;
	right_force_ = 0;
	imu_ = IMU();

	pelvis_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pelvis_pose", 1000);
	syscmd_sub_ = nh.subscribe<std_msgs::String>("/syscommand", 1000, &StateEstimator::sysCommandCb, this);

	ROS_INFO("State estimation initialized.");
	initialized_ = true;
	return true;
}

// Input
void StateEstimator::setFeetForceZ(const double& left_z, const double& right_z) {
	left_force_ = left_z;
	right_force_ = right_z;
}

void StateEstimator::setIMU(double (&orientation_no_yaw)[4], double (&angular_velocity)[3], double (&linear_acceleration)[3]) {
	imu_.orientation = Eigen::Quaterniond(orientation_no_yaw[3], orientation_no_yaw[0], orientation_no_yaw[1], orientation_no_yaw[2]);
	imu_.angular_velocity = Eigen::Vector3d(angular_velocity[0], angular_velocity[1], angular_velocity[2]);
	imu_.linear_acceleration = Eigen::Vector3d(linear_acceleration[0], linear_acceleration[1], linear_acceleration[2]);
}

void StateEstimator::setRobotTransforms(robot_tools::RobotTransforms& transforms) {
	robot_transforms_ptr_.reset(&transforms);
}

// Output
Pose StateEstimator::getPelvisPose() {
	return world_pose_;
}

std::string StateEstimator::getSupportFoot() {
	return support_foot_;
}

// Update
void StateEstimator::update() {
	if (!initialized_) return;

	if (checkSupportFootChange()) {
		setSupportFoot(otherFoot(support_foot_));
	}

	Eigen::Affine3d pelvis_to_support_foot;
	if (support_foot_ != "undefined") {
		// if we know a support foot, get the transform
		pelvis_to_support_foot = robot_transforms_ptr_->getTransform(support_foot_);
	} else {
		// otherwise just take a random foot
		pelvis_to_support_foot = robot_transforms_ptr_->getTransform(left_foot_name_);
	}
	world_pose_.orientation = pelvis_to_support_foot.linear().inverse() * ground_point_.orientation;
	Eigen::Vector3d support_foot_to_pelvis_trans = -pelvis_to_support_foot.translation();
	world_pose_.position = ground_point_.position + imu_.orientation * support_foot_to_pelvis_trans;

	publishPelvisWorldPose();
}


// Private
void StateEstimator::reset() {
	if (initialized_) {
		init(nh_);
	} else {
		ROS_WARN("Can't reset state estimation before init() was called!");
	}
}

bool StateEstimator::checkSupportFootChange() {
	double left_height = getFootHeight(left_foot_name_);
	double right_height = getFootHeight(right_foot_name_);

	double difference = left_height - right_height;

	std::string lower_foot = difference < 0 ? left_foot_name_ : right_foot_name_;
	if (std::abs(difference) > height_treshold_ && !height_treshold_passed_) {
		height_treshold_passed_ = true;
		ROS_INFO_STREAM("Height threshold passed.");
	}
	if (lower_foot != support_foot_ && height_treshold_passed_) {
		height_treshold_passed_ = false;
		return true;
	}
	return false;
}

std::string StateEstimator::otherFoot(std::string foot_name) {
	if (foot_name == right_foot_name_) {
		return left_foot_name_;
	} else {
		return right_foot_name_;
	}
}

void StateEstimator::setSupportFoot(std::string foot_name) {
	ROS_INFO_STREAM("Changing support foot to: " << foot_name);
	if (support_foot_ == "undefined") {
		// init ground position to supporting foot
		ground_point_.orientation = imu_.orientation * robot_transforms_ptr_->getTransform(foot_name).linear();
		ground_point_.position = imu_.orientation * robot_transforms_ptr_->getTransform(foot_name).translation();
		ground_point_.position.z() = 0;
		ROS_INFO_STREAM("Initializing ground_point to: " << ground_point_.position.x() << ", " << ground_point_.position.y() << ", " << ground_point_.position.z());
	} else {
		// add distance between feet to ground position
		ground_point_.orientation = (ground_point_.orientation * robot_transforms_ptr_->getTransform(support_foot_, foot_name).linear()).eval();
		ground_point_.position += robot_transforms_ptr_->getTransform(support_foot_, foot_name).translation();
		ROS_INFO_STREAM("Moving ground_point to: " << ground_point_.position.x() << ", " << ground_point_.position.y() << ", " << ground_point_.position.z());
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

	pose.pose.orientation.x = world_pose_.orientation.x();
	pose.pose.orientation.y = world_pose_.orientation.y();
	pose.pose.orientation.z = world_pose_.orientation.z();
	pose.pose.orientation.w = world_pose_.orientation.w();

	pose.pose.position.x = world_pose_.position.x();
	pose.pose.position.y = world_pose_.position.y();
	pose.pose.position.z = world_pose_.position.z();

	pelvis_pose_pub_.publish(pose);
}

void StateEstimator::sysCommandCb(const std_msgs::StringConstPtr& msg) {
	if (msg->data == "reset") {
		reset();
	} else {
		ROS_WARN_STREAM("[StateEstimator] Unknown syscommand '" << msg->data << "'.");
	}
}

} // namespace Thor
