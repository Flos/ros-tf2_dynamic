#include "tf2_dynamic_node.h"
#include <pluginlib/class_list_macros.h>

namespace image_cloud {

void
Tf2_dynamic::onInit() {
	ROS_INFO_NAMED(node_name_, "Initializing nodelet...");
	config_.x = 0;
	config_.y = 0;
	config_.z = 0;
	config_.roll = 0;
	config_.pitch = 0;
	config_.yaw = 0;
	nh = ros::NodeHandle("~");
	initParams();

	// Set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(nh));
	ReconfigureServer::CallbackType f = boost::bind(&Tf2_dynamic::reconfiguration_callback, this, _1, _2);
	reconfigure_server_->setCallback(f);
}

void
Tf2_dynamic::initParams(){
	nh.param<std::string>("parent_frame", config_.parent_frame, "");
	nh.param<std::string>("child_frame", config_.child_frame, "");
	nh.param<std::string>("name", node_name_, "tf2_dynamic_node");
	params();
}

void
Tf2_dynamic::params(){
	// Info
	ROS_INFO_NAMED(node_name_, "name:\t\t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "parent_frame:\t%s", config_.parent_frame.c_str());
	ROS_INFO_NAMED(node_name_, "child_frame:\t%s", config_.child_frame.c_str());

	// Optional
	ROS_INFO_NAMED(node_name_, "x:\t%f", config_.x);
	ROS_INFO_NAMED(node_name_, "y:\t%f", config_.y);
	ROS_INFO_NAMED(node_name_, "z:\t%f", config_.z);
	ROS_INFO_NAMED(node_name_, "roll:\t%f", config_.roll);
	ROS_INFO_NAMED(node_name_, "pitch:\t%f", config_.pitch);
	ROS_INFO_NAMED(node_name_, "yaw:\t%f", config_.yaw);

	if(config_.parent_frame.empty()) ROS_WARN_NAMED(node_name_, "Cannot publish transform without parent frame");
	if(config_.child_frame.empty()) ROS_WARN_NAMED(node_name_, "Cannot publish transform without child frame");

	transformStamped.header.frame_id = config_.parent_frame;
	transformStamped.child_frame_id = config_.child_frame;
	transformStamped.transform.translation.x = config_.x;
	transformStamped.transform.translation.y = config_.y;
	transformStamped.transform.translation.z = config_.z;
	tf2::Quaternion q;
	q.setRPY(config_.roll, config_.pitch, config_.yaw);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();
}

void
Tf2_dynamic::loop(){
	ROS_INFO_NAMED(node_name_, "loop start");
	ros::Rate rate(50);
	while(nh.ok()){
		try{;
			transformStamped.header.stamp = ros::Time::now();
			br.sendTransform(transformStamped);
		}catch (tf2::TransformException &ex) {
		      ROS_WARN("%s",ex.what());
		      ros::Duration(1).sleep();
		      continue;
		}
		ros::spinOnce();
		rate.sleep();
	}
	ROS_WARN_NAMED(node_name_, "loop end");
}

void
Tf2_dynamic::reconfiguration_callback(Config &config, uint32_t level){
	config_ = config;
	params();
}

Tf2_dynamic::~Tf2_dynamic(){
	}

} /* end namespace */


int main(int argc, char **argv){
	ros::init(argc, argv, "manual_calibration");
	image_cloud::Tf2_dynamic manual_transform;
	ROS_INFO_NAMED("maunal_calibration", "start");
	manual_transform.onInit();
	manual_transform.loop();
	ROS_INFO_NAMED("maunal_calibration", "done");
}
