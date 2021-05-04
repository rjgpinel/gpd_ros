/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Andreas ten Pas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef GRASP_DETECTION_SERVER_PRL_H_
#define GRASP_DETECTION_SERVER_PRL_H_

// system
#include <algorithm>
#include <memory>
#include <vector>

// ROS
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

// this project (services)
#include <gpd_ros/detect_grasps_prl.h>

// this project (messages)
#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>

// this project (headers)
#include <gpd_ros/grasp_messages.h>
#include <gpd_ros/grasp_plotter.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;


class GraspDetectionServerPRL{
 public:
  /**
   * \brief Constructor.
   * \param node the ROS node
   */
  GraspDetectionServerPRL(ros::NodeHandle& node);

  /**
   * \brief Destructor.
   */
  ~GraspDetectionServerPRL()
    {
      delete cloud_camera_;
      delete grasp_detector_;
      delete rviz_plotter_;
    }


  bool detectGrasps(gpd_ros::detect_grasps_prl::Request& req, gpd_ros::detect_grasps_prl::Response& res);

  std::vector<std::unique_ptr<gpd::candidate::Hand>> detectGraspPoses();

 private:

  /**
   * \brief Callback function for the ROS topic that contains the input point cloud.
   * \param msg the incoming ROS message
  */
  void cloud_callback(const sensor_msgs::PointCloud2& msg);

  Eigen::Vector3d view_point_; ///< (input) view point of the camera onto the point cloud

  gpd::util::Cloud* cloud_camera_; ///< stores point cloud with (optional) camera information and surface normals
  std_msgs::Header cloud_camera_header_; ///< stores header of the point cloud

  bool has_cloud_; ///< status variable for received (input) messages
  std::string frame_; ///< point cloud frame
  ros::Subscriber cloud_sub_; ///< ROS subscriber for point cloud messages
  ros::Publisher grasps_rviz_pub_; ///< ROS publisher for grasps in rviz (visualization)

  bool use_rviz_; ///< if rviz is used for visualization instead of PCL
  std::vector<double> workspace_; ///< workspace limits

  gpd::GraspDetector* grasp_detector_; ///< used to run the GPD algorithm
  GraspPlotter* rviz_plotter_; ///< used to plot detected grasps in rviz

};




#endif /* GRASP_DETECTION_SERVER_PRL_H_ */
