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
  yaw_(0.0),
  id_(0)
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
  nh.param("height_treshold", height_treshold_, 0.02); // 0.05
  nh.param("ankle_z_offset", ankle_z_offset_, 0.0);
  nh.param("tf_odom_frame", tf_odom_frame_, std::string("world"));

  double roll, pitch, yaw;
  nh.param("imu_world_roll_offset", roll, 0.0);
  nh.param("imu_world_pitch_offset", pitch, 0.0);
  nh.param("imu_world_yaw_offset", yaw, 0.0);
  imu_world_rotation_offset_ = rpyToRot(roll, pitch, yaw);

  std::vector<double> com_offset;
  nh.param("com_offset", com_offset, std::vector<double>(3, 0));
  if (com_offset.size() != 3) {
    ROS_ERROR_STREAM("[StateEstimation] Com offset needs to have size 3. Received size " << com_offset.size());
    com_offset_ = Eigen::Vector3d::Zero();
  } else {
    for (unsigned int i = 0; i < 3; i++) {
      com_offset_(i) = com_offset[i];
    }
  }

	left_force_ = 0;
	right_force_ = 0;
	imu_ = IMU();

	pelvis_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pelvis_pose", 1000);
	ground_point_pub_ = nh.advertise<geometry_msgs::PoseStamped>("ground_point", 1000);
  footstep_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("footsteps", 1000);
	com_pub_ = nh.advertise<geometry_msgs::PoseStamped>("com", 1000);
  ground_com_pub_ = nh.advertise<geometry_msgs::PoseStamped>("ground_com", 1000);
	syscmd_sub_ = nh.subscribe<std_msgs::String>("/syscommand", 1000, &StateEstimator::sysCommandCb, this);

  imu_orientation_pub_ = nh.advertise<geometry_msgs::PoseStamped>("imu_orientation", 1000);

	ROS_INFO("State estimation initialized.");
	initialized_ = true;
	resetted_ = false;
	if (reset_on_start) {
		reset();
	}
	return true;
}

void StateEstimator::reset() {
	if (initialized_) {
		resetted_ = true;
    height_treshold_passed_ = false;
		// Init state estimation
		support_foot_ = left_foot_name_; // Starting in double support
		world_pose_ = Pose();
		ground_point_.orientation = Eigen::Quaterniond::Identity();
    ground_point_.position = imuToRot(imu_) * robot_transforms_ptr_->getTransform(pelvis_name_, left_foot_name_).translation();
		ground_point_.position.z() = ankle_z_offset_;
		yaw_ = 0.0;
		ROS_INFO_STREAM("[StateEstimator] Reset.");
	} else {
		ROS_WARN("Can't reset state estimation before init() was called!");
	}
}

// Input
void StateEstimator::setFeetForceZ(const double& left_z, const double& right_z) {
	left_force_ = left_z;
	right_force_ = right_z;
}

void StateEstimator::setIMU(double roll, double pitch, double yaw) {
	imu_.roll = roll;
	imu_.pitch = pitch;
	imu_.yaw = 0;
}

void StateEstimator::setIMU(const sensor_msgs::ImuConstPtr& imu_ptr) {
  // we get world -> imu (^W R_I)
  Eigen::Quaterniond imu(imu_ptr->orientation.w, imu_ptr->orientation.x, imu_ptr->orientation.y, imu_ptr->orientation.z); // ^W R_I

  // rotate to pelvis frame
  Eigen::Affine3d imu_to_pelvis = robot_transforms_ptr_->getTransform(imu_ptr->header.frame_id, pelvis_name_); // ^I R_P
  Eigen::Quaterniond imu_pelvis(imu * imu_to_pelvis.linear()); // ^W R_P = ^W R_I * ^I R_P
  imu_pelvis = imu_world_rotation_offset_ * imu_pelvis; // from IMU world convention (NED, ..) to our convention (ENU)

//   we want to ignore yaw
  Eigen::Vector3d rpy = rotToRpy(imu_pelvis.toRotationMatrix());
  imu_.roll = rpy(0);
  imu_.pitch = rpy(1);
  imu_.yaw = 0;

  // publish IMU as pelvis -> world rotation without yaw
  Eigen::Quaterniond imu_pelvis_no_yaw = rpyToRot(rpy(0), rpy(1), 0).inverse();

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now(); // overwrite time for debugging purposes
  pose_msg.header.frame_id = pelvis_name_;
  pose_msg.pose.orientation.w = imu_pelvis_no_yaw.w();
  pose_msg.pose.orientation.x = imu_pelvis_no_yaw.x();
  pose_msg.pose.orientation.y = imu_pelvis_no_yaw.y();
  pose_msg.pose.orientation.z = imu_pelvis_no_yaw.z();

  imu_orientation_pub_.publish(pose_msg);
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

Pose StateEstimator::getCOMinFootFrame() {
	Pose com_pose;
  com_pose.position = world_pose_.position - ground_point_.position + (world_pose_.orientation * com_offset_);
	com_pose.orientation = ground_point_.orientation.inverse() * world_pose_.orientation;
	return com_pose;
}

Eigen::Vector6d getCOMinFootFrameRPY() {
	Eigen::Vector6d pose;
	return pose;
}

// Update
void StateEstimator::update(ros::Time current_time) {
	if (!initialized_ || !resetted_) return;

	if (checkSupportFootChange()) {
		setSupportFoot(otherFoot(support_foot_));
	}

  Eigen::Affine3d pelvis_to_support_foot = robot_transforms_ptr_->getTransform(pelvis_name_, support_foot_);

	double roll, pitch, yaw;
	roll = imu_.roll;
	pitch = imu_.pitch;
	yaw = yaw_ + rotToRpy(pelvis_to_support_foot.rotation().inverse())(2);

	world_pose_.orientation = rpyToRot(roll, pitch, yaw);

	// translate the pelvis by distance between (fixed) foot frame and actual support foot position in world frame
	world_pose_.position += ground_point_.position - (world_pose_.orientation * pelvis_to_support_foot.translation() + world_pose_.position);

	publishPelvisWorldPose(current_time);
	publishGroundPoint(current_time);
	publishCOM(current_time);
}

void StateEstimator::update() {
	update(ros::Time::now());
}

// Private
bool StateEstimator::checkSupportFootChange() {
  double left_height = getFootHeight(left_foot_name_);
	double right_height = getFootHeight(right_foot_name_);

  double difference = left_height - right_height;
  // ROS_INFO_STREAM("Height difference: " << difference);

	std::string lower_foot = difference < 0 ? left_foot_name_ : right_foot_name_;
	if (std::abs(difference) > height_treshold_ && !height_treshold_passed_) {
		height_treshold_passed_ = true;
    // ROS_INFO("Threshold passed.");
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
  Eigen::Affine3d foot_to_foot = robot_transforms_ptr_->getTransform(support_foot_, foot_name);

  // add distance between feet to ground position
  ground_point_.position += rpyToRot(0, 0, yaw_) * foot_to_foot.translation();
  ground_point_.position.z() =  + ankle_z_offset_; // fix foot to ground again

  yaw_ += rotToRpy(foot_to_foot.rotation())(2);
	ground_point_.orientation = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());

  // ROS_INFO_STREAM("Switching support foot to " << foot_name << ".");
	support_foot_ = foot_name;
}


double StateEstimator::getFootHeight(std::string foot_name) {
  Eigen::Affine3d pose	= robot_transforms_ptr_->getTransform(pelvis_name_, foot_name);
	Eigen::Vector3d position = pose.translation();
	Eigen::Vector3d world_position = imuToRot(imu_) * position;
  //ROS_INFO_STREAM("Foot height: " << foot_name << ": " << world_position.z());
	return world_position.z();
}

void StateEstimator::publishPose(const Pose& pose, const ros::Publisher& pub, ros::Time current_time, std::string frame, bool add_to_array) {
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = frame;
	pose_msg.header.stamp = current_time;

	pose_msg.pose.orientation.x = pose.orientation.x();
	pose_msg.pose.orientation.y = pose.orientation.y();
	pose_msg.pose.orientation.z = pose.orientation.z();
	pose_msg.pose.orientation.w = pose.orientation.w();

	pose_msg.pose.position.x = pose.position.x();
	pose_msg.pose.position.y = pose.position.y();
	pose_msg.pose.position.z = pose.position.z();

	pub.publish(pose_msg);
  if (add_to_array) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker pose_marker;
    pose_marker.id = id_++;
    pose_marker.header.frame_id = frame;
    pose_marker.header.stamp = current_time;
    pose_marker.pose = pose_msg.pose;
    pose_marker.pose.position.z -= ankle_z_offset_;
    geometry_msgs::Vector3 scale;
    scale.x = 0.24; scale.y = 0.145; scale.z = 0.015;
    pose_marker.scale = scale;

    pose_marker.color.a = 1.0;
    pose_marker.color.r = 0.0;
    pose_marker.color.g = 1.0;
    pose_marker.color.b = 0.0;

    pose_marker.type = visualization_msgs::Marker::CUBE;
    pose_marker.action = visualization_msgs::Marker::ADD;
    marker_array.markers.push_back(pose_marker);
    footstep_vis_pub_.publish(marker_array);
  }
}

void StateEstimator::publishPelvisWorldPose(ros::Time current_time) {
  publishPose(world_pose_, pelvis_pose_pub_, current_time, tf_odom_frame_, false);
}

void StateEstimator::publishGroundPoint(ros::Time current_time) {
  publishPose(ground_point_, ground_point_pub_, current_time, tf_odom_frame_, true);
}

void StateEstimator::publishCOM(ros::Time current_time) {
	Pose com_pose;
  com_pose.position = world_pose_.position - ground_point_.position + (world_pose_.orientation * com_offset_);
	com_pose.orientation = ground_point_.orientation.inverse() * world_pose_.orientation;
	publishPose(com_pose, com_pub_, current_time, "footstep_frame");

  Pose ground_com;
  ground_com.position = world_pose_.position + (world_pose_.orientation * com_offset_);
  ground_com.orientation = world_pose_.orientation;
  ground_com.position.z() = 0;
  publishPose(ground_com, ground_com_pub_, current_time, "world");
}

void StateEstimator::sysCommandCb(const std_msgs::StringConstPtr& msg) {
  if (msg->data == "reset_pose_estimation") {
		reset();
  }
}

Eigen::Quaterniond StateEstimator::imuToRot(IMU imu) const {
	return rpyToRot(imu.roll, imu.pitch, imu.yaw);
}

Eigen::Quaterniond StateEstimator::rpyToRot(double roll, double pitch, double yaw) const {
	KDL::Rotation kdl_rot = KDL::Rotation::RPY(roll, pitch, yaw);
	Eigen::Quaterniond quat;
	kdl_rot.GetQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
	return quat;
}

Eigen::Vector3d StateEstimator::rotToRpy(Eigen::Matrix3d rot) const {
	Eigen::Quaterniond rot_quad(rot);
	KDL::Rotation kdl_rot = KDL::Rotation::Quaternion(rot_quad.x(), rot_quad.y(), rot_quad.z(), rot_quad.w());
	double roll, pitch, yaw;
	kdl_rot.GetRPY(roll, pitch, yaw);
	return Eigen::Vector3d(roll, pitch, yaw);
}

bool StateEstimator::isInitialized() const {
  return initialized_;
}

} // namespace Thor
