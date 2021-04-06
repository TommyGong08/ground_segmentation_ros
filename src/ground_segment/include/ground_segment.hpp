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

#ifndef  GROUND_SEGMENT_HPP
#define GROUND_SEGMENT_HPP

//#include "fsd_common_msgs/ConeDetections.h"
#include "geometry_msgs/Point32.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <chrono>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <velodyne_pointcloud/point_types.h>

#include <Eigen/Dense>
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

namespace scan_line_run
{
  /** Euclidean Velodyne coordinate, including intensity and ring number, and label. */
  struct PointXYZIRL
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    uint16_t label;                     ///< point label
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace scan_line_run

#define SLRPointXYZIRL scan_line_run::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCloud<SLRPointXYZIRL>
// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(scan_line_run::PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label))

namespace ns_ground_segment {

class GroundSegment {

 public:
  // Constructor
  GroundSegment(ros::NodeHandle &nh);

  // Getters
  sensor_msgs::PointCloud2 getGroundPoint();
  sensor_msgs::PointCloud2 getGroundlessPoint(); 
  sensor_msgs::PointCloud2 getAllPoints();

  bool is_ok();

  // Setters
  void setRawLidar(sensor_msgs::PointCloud2 msg);

  void runAlgorithm();

 private:
    double sensor_height_;
    int num_seg_;
    int num_iter_;
    int num_lpr_;
    double th_seeds_;
    double th_dist_;

    float d_;
    MatrixXf normal_;
    float th_dist_d_;
    
    ros::NodeHandle &nh_;

    void loadParameters();

    bool getRawLidar, is_ok_flag_;
    bool is_cone_reconstruct_;

    sensor_msgs::PointCloud cluster_;
    sensor_msgs::PointCloud2 raw_pc2_;

    pcl::PointCloud<VPoint> laserCloudIn;
    pcl::PointCloud<VPoint> laserCloudIn_org;
    SLRPointXYZIRL point;

    void velodyne_callback_(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    void estimate_plane_(void);
    void extract_initial_seeds_(const pcl::PointCloud<VPoint>& p_sorted);
    int name_index = 0;

    // Model parameter for ground plane fitting
    // The ground plane model is: ax+by+cz+d=0
    // Here normal:=[a,b,c], d=d
    // th_dist_d_ = threshold_dist - d 

  
    //publish
    sensor_msgs::PointCloud2 ground_msg;
    sensor_msgs::PointCloud2 groundless_msg;
    sensor_msgs::PointCloud2 all_points_msg;
};
} // namespace ns_ground_segment

#endif // GROUND_SEGMENT_HPP
