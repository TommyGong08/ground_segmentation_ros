/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2019:
     - chentairan <killasipilin@gmail.com>

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

#ifndef GROUND_SEGMENT_HANDLE_HPP
#define GROUND_SEGMENT_HANDLE_HPP

#include "fsd_common_msgs/ConeDetections.h"
#include "ground_segment.hpp"

namespace ns_ground_segment {

class GroundSegmentHandle {
 public:
  // Constructor
  GroundSegmentHandle(ros::NodeHandle &nodeHandle);

//  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();

 private:
  ros::NodeHandle node_handle_;
  ros::Subscriber points_node_sub_;
  ros::Publisher ground_points_pub_;
  ros::Publisher groundless_points_pub_;
  ros::Publisher all_points_pub_;

  void rawLidarCallback(const sensor_msgs::PointCloud2 &msg);

  std::string raw_lidar_topic_name_;

  int sensor_model_;
  double sensor_height_;
  int num_seg_;
  int num_iter_;
  int num_lpr_;
  double th_seeds_;
  double th_dist_;

  int node_rate_;

  GroundSegment ground_segment_;

  bool get_new_msg_ = false;

};
}

#endif //GROUND_SEGMENT_HANDLE_HPP
