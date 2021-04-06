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

#include "lidar_cluster.hpp"
#include <ros/ros.h>
#include <sstream>
#include <utility>

namespace ns_ground_segment {
// Constructor
LidarCluster::LidarCluster(ros::NodeHandle &nh) : nh_(nh) { loadParameters(); };

// load Param
void LidarCluster::loadParameters() {
  getRawLidar = false;
  is_ok_flag_ = false;
}

// Getters
sensor_msgs::PointCloud GroundSegment::getGroundPoint() { return cluster_; }


sensor_msgs::PointCloud2 GroundSegment::getGroundlessPoint() {
  return filter_ground_;
}
sensor_msgs::PointCloud2 GroundSegment::getAllPoints() { return filter_cones_; }

bool GroundSegment::is_ok() { return is_ok_flag_; }

// Setters
void GroundSegment::setRawLidar(sensor_msgs::PointCloud2 msg){
  raw_pc2_ = std::move(msg);
  getRawLidar = true;
}

void GroundSegment::runAlgorithm() {
    if(raw_pc2.fields.isempty() || !getRawLidar){
      getRawLidar = false;
      return;
    }





}

void GroundPlaneFit::estimate_plane_(void){
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    // Singular Value Decomposition: SVD
    JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose()*seeds_mean)(0,0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;

    // return the equation parameters
}


/*
    @brief Extract initial seeds of the given pointcloud sorted segment.
    This function filter ground seeds points accoring to heigt.
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud
    
    @param ::num_lpr_: num of LPR points
    @param ::th_seeds_: threshold distance of seeds
    @param ::

*/
void GroundPlaneFit::extract_initial_seeds_(const pcl::PointCloud<VPoint>& p_sorted){
    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    // Calculate the mean height value.
    for(int i=0;i<p_sorted.points.size() && cnt<num_lpr_;i++){
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt!=0?sum/cnt:0;// in case divide by 0
    g_seeds_pc->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for(int i=0;i<p_sorted.points.size();i++){
        if(p_sorted.points[i].z < lpr_height + th_seeds_){
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}

} // namespace ns_ground_segment
