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
  got_joint_states = true;
	for (unsigned int i = 0; i < joint_state_ptr->name.size(); i++) {
		transforms_ptr->updateState(joint_state_ptr->name[i], joint_state_ptr->position[i]);
	}
  if (estimator_ptr->isInitialized()) {
    estimator_ptr->update(joint_state_ptr->header.stamp);
  }
}

void imu_cb(const sensor_msgs::ImuConstPtr& imu_ptr) {
  got_imu = true;
  if (estimator_ptr->isInitialized()) {
    estimator_ptr->setIMU(imu_ptr);
  }
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "state_estimator_test_node");

	ros::NodeHandle nh("state_estimator");

	transforms_ptr.reset(new robot_tools::RobotTransforms);
	transforms_ptr->init();

	estimator_ptr.reset(new robot_tools::StateEstimator);
	estimator_ptr->setRobotTransforms(transforms_ptr);

  joint_state_sub = nh.subscribe("joint_states", 1000, &joint_state_cb);
  imu_sub = nh.subscribe("pelvis_imu", 1000, &imu_cb);

  while (!(got_imu && got_joint_states) && ros::ok()) {
		usleep(8000);
		ros::spinOnce();
	}

	estimator_ptr->init(nh, true);

	ros::spin();
} 
