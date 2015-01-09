/*
 * calibration_manual.h
 *
 *  Created on: 11.08.2014
 *      Author: fnolden
 */

#ifndef CALIBRATION_MANUAL_H_
#define CALIBRATION_MANUAL_H_

#include "ros/ros.h"
#include <sstream>
#include <string.h>
#include <map>
#include <iostream>
#include <functional>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <dynamic_reconfigure/server.h>
#include <tf2_dynamic/tf2_dynamic_nodeConfig.h>

namespace tf2_dynamic {

class Tf2_dynamic {
	typedef tf2_dynamic::tf2_dynamic_nodeConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
public:
	virtual void onInit();
	virtual ~Tf2_dynamic();
	virtual void loop();
	virtual void reconfiguration_callback(Config &config, uint32_t level);
	virtual void initParams();
	virtual void params();
private:
	ros::NodeHandle nh;
	std::string node_name_;

	tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	Config config_;
	boost::shared_ptr<ReconfigureServer> reconfigure_server_;
};
} /* end namespace */

#endif /* calibration_manual */
