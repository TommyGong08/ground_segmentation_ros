/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2021:
     - gonghailong <GongHailong2020@126.com>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "ground_segment_handle.hpp"

namespace ns_ground_segment {

// Constructor
GroundSegmentHandle::GroundSegmentHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    ground_segment_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int GroundSegmentHandle::getNodeRate() const { return node_rate_; }

// Methods
void GroundSegmentHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  node_handle_.param<std::string>("raw_lidar_topic_name", raw_lidar_topic_name_, "/velodyne_points");
  ROS_INFO("Input Point Cloud: %s", point_topic_.c_str());

  node_handle_.param("sensor_model", sensor_model_, 32);
  ROS_INFO("Sensor Model: %d", sensor_model_);

  node_handle_.param("sensor_height", sensor_height_, 2.5);
  ROS_INFO("Sensor Height: %f", sensor_height_);

  node_handle_.param("num_seg", num_seg_, 1);
  ROS_INFO("Num of Segments: %d", num_seg_);

  node_handle_.param("num_iter", num_iter_, 3);
  ROS_INFO("Num of Iteration: %d", num_iter_);

  node_handle_.param("num_lpr", num_lpr_, 20);
  ROS_INFO("Num of LPR: %d", num_lpr_);

  node_handle_.param("th_seeds", th_seeds_, 1.2);
  ROS_INFO("Seeds Threshold: %f", th_seeds_);

  node_handle_.param("th_dist", th_dist_, 0.3);
  ROS_INFO("Distance Threshold: %f", th_dist_);

  node_handle_.param<std::string>("no_ground_point_topic", no_ground_topic, "/points_no_ground");
  ROS_INFO("No Ground Output Point Cloud: %s", no_ground_topic.c_str());

  node_handle_.param<std::string>("ground_point_topic", ground_topic, "/points_ground");
  ROS_INFO("Only Ground Output Point Cloud: %s", ground_topic.c_str());

   if (!node_handle_.param("node_rate", node_rate_, 50)) {
  ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }

}

void GroundSegmentHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  rawLidarSubscriber_ = node_handle_.subscribe(raw_lidar_topic_name_, 1, &GroundSegmentHandle::rawLidarCallback, this);
}

void GroundSegmentHandle::publishToTopics() {
  ROS_INFO("publish to topics");
    groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 2);
    ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic, 2);
    all_points_pub_ =  node_handle_.advertise<sensor_msgs::PointCloud2>("/all_points", 2);
}

void GroundSegmentHandle::run() {
  if(get_new_msg_) {
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    //runAlgorithm
    ground_segment_.runAlgorithm();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t2).count();
    std::cout<<"time:"<<time_round<<std::endl;
    sendMsg();
    get_new_msg_ = false;
  }
}

void LidarClusterHandle::sendMsg() {
  if(!ground_segment_.is_ok())  return;
    ground_points_pub_.publish(ground_segment_.getGroundPoint());
    groundless_points_pub_.publish(ground_segment_.getGroundlessPoint());
    all_points_pub_.publish(ground_segment_.getAllPoints());
}

void LidarClusterHandle::rawLidarCallback(const sensor_msgs::PointCloud2 &msg) {
  ground_segment_.setRawLidar(msg);
  get_new_msg_ = true;
}
}