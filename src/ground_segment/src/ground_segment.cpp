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

#include "ground_segment.hpp"
#include <ros/ros.h>
#include <sstream>
#include <utility>

pcl::PointCloud<VPoint>::Ptr g_seeds_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_ground_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_not_ground_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<SLRPointXYZIRL>::Ptr g_all_pc(new pcl::PointCloud<SLRPointXYZIRL>());


namespace ns_ground_segment {

// Constructor
GroundSegment::GroundSegment(ros::NodeHandle &nh) : nh_(nh) { loadParameters(); };

// load Param
void GroundSegment::loadParameters() {
  getRawLidar = false;
  is_ok_flag_ = false;
 sensor_model_ = 32;
 sensor_height_ = 2.5;
 num_seg_ = 1;
 num_iter_ = 3;
 num_lpr_ = 20;
 th_seeds_ = 1.2 ;
 th_dist_ = 0.3;
}

// Getters
sensor_msgs::PointCloud2 GroundSegment::getGroundPoint() { return ground_msg; }

sensor_msgs::PointCloud2 GroundSegment::getGroundlessPoint() {
  return groundless_msg;
}
sensor_msgs::PointCloud2 GroundSegment::getAllPoints() { return all_points_msg; }

bool GroundSegment::is_ok() { return is_ok_flag_; }

// Setters
void GroundSegment::setRawLidar(const sensor_msgs::PointCloud2 msg){
  std::cout << "GET RAW POINT CLOUD\n " << std::endl; 
  raw_pc2_ = std::move(msg);
  getRawLidar = true;
}

bool point_cmp(VPoint a, VPoint b){
    return a.z<b.z;
}

void GroundSegment::runAlgorithm() {
    if(raw_pc2_.fields.empty() || !getRawLidar){
      getRawLidar = false;
      return;
    }
     getRawLidar = false;

// 1.Msg to pointcloud
    std::cout << "1111111111111111" << std::endl;
    pcl::fromROSMsg(raw_pc2_, laserCloudIn);
    pcl::fromROSMsg(raw_pc2_, laserCloudIn_org);
    // For mark ground points and hold all points
    for(size_t i=0;i<laserCloudIn.points.size();i++){
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        point.ring = laserCloudIn.points[i].ring;
        point.label = 0u;// 0 means uncluster
        g_all_pc->points.push_back(point);
    }
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);

    // 2.Sort on Z-axis value.
    std::cout << "22222222222222" << std::endl;
    sort(laserCloudIn.points.begin(),laserCloudIn.end(),point_cmp);

    // 3.Error point removal
    // As there are some error mirror reflection under the ground, 
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    std::cout << "33333333333333" << std::endl;
    pcl::PointCloud<VPoint>::iterator it = laserCloudIn.points.begin();
    for(int i=0;i<laserCloudIn.points.size();i++){
        if(laserCloudIn.points[i].z < -1.5 * sensor_height_){
            it++;
        }else{
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(),it);

    // 4. Extract init ground seeds.
    std::cout << "444444444444" << std::endl;
    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc;
    
    // 5. Ground plane fitter mainloop
    std::cout << "5555555555" << std::endl;
    for(int i=0;i<num_iter_;i++){
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        //pointcloud to matrix
        MatrixXf points(laserCloudIn_org.points.size(),3);
        int j =0;
        for(auto p:laserCloudIn_org.points){
            points.row(j++)<<p.x,p.y,p.z;
        }
        // ground plane model
        VectorXf result = points*normal_;
        // threshold filter
        for(int r=0;r<result.rows();r++){
            if(result[r]<th_dist_d_){
                g_all_pc->points[r].label = 1u;// means ground
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }else{
                g_all_pc->points[r].label = 0u;// means not ground and non clusterred
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }

     // publish ground points
    pcl::toROSMsg(*g_ground_pc, ground_msg);//
    ground_msg.header.stamp = raw_pc2_.header.stamp;
    ground_msg.header.frame_id = raw_pc2_.header.frame_id;
    // publish not ground points
    pcl::toROSMsg(*g_not_ground_pc, groundless_msg);
    groundless_msg.header.stamp = raw_pc2_.header.stamp;
    groundless_msg.header.frame_id = raw_pc2_.header.frame_id;
    // publish all points
    pcl::toROSMsg(*g_all_pc, all_points_msg);
    all_points_msg.header.stamp = raw_pc2_.header.stamp;
    all_points_msg.header.frame_id = raw_pc2_.header.frame_id;
    g_all_pc->clear();

    is_ok_flag_ = true;
}

void GroundSegment::estimate_plane_(void){
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
void GroundSegment::extract_initial_seeds_(const pcl::PointCloud<VPoint>& p_sorted){
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
