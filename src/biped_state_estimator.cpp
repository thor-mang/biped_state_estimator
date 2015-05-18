#include <biped_state_estimator/biped_state_estimator.h>

#include <kdl/frames.hpp>

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
	initialized_(false),
	resetted_(false),
	ankle_z_offset_(0.0),
	yaw_(0.0)
{}

bool StateEstimator::init(ros::NodeHandle nh, bool reset_on_start) {
	if (!robot_transforms_ptr_) {
		ROS_ERROR("[StateEstimation] Robot Transforms wasn't set. Can't initialize");
		return false;
	}
	// Load params
	nh_ = nh;
	nh.param("pelvis_name", pelvis_name_, std::string("pelvis"));
	nh.param("right_foot_name", right_foot_name_, std::string("r_foot"));
	nh.param("left_foot_name", left_foot_name_, std::string("l_foot"));
	nh.param("height_treshold", height_treshold_, 0.05);
	nh.param("ankle_z_offset", ankle_z_offset_, 0.118);

	left_force_ = 0;
	right_force_ = 0;
	imu_ = IMU();

	pelvis_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pelvis_pose", 1000);
	ground_point_pub_ = nh.advertise<geometry_msgs::PoseStamped>("ground_point", 1000);
	syscmd_sub_ = nh.subscribe<std_msgs::String>("/syscommand", 1000, &StateEstimator::sysCommandCb, this);

	ROS_INFO("State estimation initialized.");
	initialized_ = true;
	resetted_ = false;
	if (reset_on_start) {
		reset();
	}
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

void StateEstimator::setRobotTransforms(boost::shared_ptr<RobotTransforms> transforms_ptr) {
	robot_transforms_ptr_ = transforms_ptr;
}

// Output
Pose StateEstimator::getPelvisPose() {
	return world_pose_;
}

std::string StateEstimator::getSupportFoot() {
	return support_foot_;
}

// Update
void StateEstimator::update(ros::Time current_time) {
	if (!initialized_ || !resetted_) return;

	if (checkSupportFootChange()) {
		setSupportFoot(otherFoot(support_foot_));
	}

	Eigen::Affine3d pelvis_to_support_foot = robot_transforms_ptr_->getTransform(support_foot_);
	world_pose_.orientation = ground_point_.orientation * pelvis_to_support_foot.linear().inverse() * imu_.orientation;

	Eigen::Vector3d support_foot_to_pelvis_trans = -pelvis_to_support_foot.translation();
	world_pose_.position = ground_point_.position + support_foot_to_pelvis_trans;//ground_point_.position + imu_.orientation * support_foot_to_pelvis_trans;

	publishPelvisWorldPose(current_time);
	publishGroundPoint(current_time);
}

void StateEstimator::update() {
	update(ros::Time::now());
}


// Private
void StateEstimator::reset() {
	if (initialized_) {
		resetted_ = true;
		height_treshold_passed_ = false;
		// Init state estimation
		support_foot_ = left_foot_name_; // Starting in double support
		world_pose_ = Pose();
		ground_point_.orientation = Eigen::Quaterniond::Identity();
		// ROS_INFO_STREAM("Initializing orientation to: " << ground_point_.orientation.x() << ", " <<  ground_point_.orientation.y() << ", " <<  ground_point_.orientation.z() << ", " <<  ground_point_.orientation.w());
		ground_point_.position = imu_.orientation * robot_transforms_ptr_->getTransform(left_foot_name_).translation();
		ground_point_.position.z() = ankle_z_offset_;
		yaw_ = 0.0;
		ROS_INFO_STREAM("[StateEstimator] Reset.");
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
		// ROS_INFO_STREAM("Height threshold passed.");
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
	// add distance between feet to ground position
	Eigen::Affine3d foot_to_foot = robot_transforms_ptr_->getTransform(support_foot_, foot_name);
	ground_point_.position += foot_to_foot.translation();
	ground_point_.position.z() =  + ankle_z_offset_; // fix foot to ground again

	// extract yaw
	Eigen::Quaterniond foot_rot_quad(foot_to_foot.rotation());
	KDL::Rotation rot = KDL::Rotation::Quaternion(foot_rot_quad.x(), foot_rot_quad.y(), foot_rot_quad.z(), foot_rot_quad.w());
	double roll, pitch, yaw;
	rot.GetRPY(roll, pitch, yaw);
	yaw_ += yaw;

	ground_point_.orientation = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());

	//Eigen::Vector3d d = robot_transforms_ptr_->getTransform(support_foot_, foot_name).translation();
	ROS_INFO_STREAM("Switching support foot from " << support_foot_ << " to " << foot_name << "."); // << std::endl <<
	//								"Displacement: " << std::endl << d.x() << ", " << d.y() << ", " << d.z());
	ROS_INFO_STREAM("rpy: " << roll << ", " << pitch << ", " << yaw);
	support_foot_ = foot_name;
}


double StateEstimator::getFootHeight(std::string foot_name) {
	Eigen::Affine3d pose	= robot_transforms_ptr_->getTransform(foot_name);
	Eigen::Vector3d position = pose.translation();
	Eigen::Vector3d world_position = imu_.orientation * position;
	return world_position.z();
}

void StateEstimator::publishPose(const Pose& pose, const ros::Publisher& pub, ros::Time current_time) const {
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = "world";
	pose_msg.header.stamp = current_time;

	pose_msg.pose.orientation.x = pose.orientation.x();
	pose_msg.pose.orientation.y = pose.orientation.y();
	pose_msg.pose.orientation.z = pose.orientation.z();
	pose_msg.pose.orientation.w = pose.orientation.w();

	pose_msg.pose.position.x = pose.position.x();
	pose_msg.pose.position.y = pose.position.y();
	pose_msg.pose.position.z = pose.position.z();

	pub.publish(pose_msg);
}

void StateEstimator::publishPelvisWorldPose(ros::Time current_time) {
	publishPose(world_pose_, pelvis_pose_pub_, current_time);
}

void StateEstimator::publishGroundPoint(ros::Time current_time) {
	publishPose(ground_point_, ground_point_pub_, current_time);
}

void StateEstimator::sysCommandCb(const std_msgs::StringConstPtr& msg) {
	if (msg->data == "reset") {
		reset();
	} else {
		ROS_WARN_STREAM("[StateEstimator] Unknown syscommand '" << msg->data << "'.");
	}
}

} // namespace Thor
