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
  ROS_INFO("Input Point Cloud: %s", raw_lidar_topic_name_.c_str());

  node_handle_.param<std::string>("no_ground_point_topic", no_ground_topic_name_, "/points_no_ground");
  ROS_INFO("No Ground Output Point Cloud: %s", no_ground_topic_name_.c_str());

  node_handle_.param<std::string>("ground_point_topic", ground_topic_name_, "/points_ground");
  ROS_INFO("Only Ground Output Point Cloud: %s", ground_topic_name_.c_str());

 node_handle_.param<int>("node_rate", node_rate_, 50);
  ROS_INFO("Load node_rate. Standard value is: %d" ,node_rate_);

}

void GroundSegmentHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  rawLidarSubscriber_ = node_handle_.subscribe(raw_lidar_topic_name_, 1, &GroundSegmentHandle::rawLidarCallback, this);
}

void GroundSegmentHandle::publishToTopics() {
  ROS_INFO("publish to topics");
    groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(no_ground_topic_name_, 2);
    ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic_name_, 2);
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

void GroundSegmentHandle::sendMsg() {
  if(!ground_segment_.is_ok())  return;
    ground_points_pub_.publish(ground_segment_.getGroundPoint());
    groundless_points_pub_.publish(ground_segment_.getGroundlessPoint());
    all_points_pub_.publish(ground_segment_.getAllPoints());
}

void GroundSegmentHandle::rawLidarCallback(const sensor_msgs::PointCloud2 &msg) {
  ground_segment_.setRawLidar(msg);
  get_new_msg_ = true;
}
}