#include <gpd_ros/grasp_detection_server_prl.h>

GraspDetectionServerPRL::GraspDetectionServerPRL(ros::NodeHandle& node){

  std::string cfg_file;
  node.param("config_file", cfg_file, std::string(""));
  grasp_detector_ = new gpd::GraspDetector(cfg_file);

  std::string rviz_topic;
  node.param("rviz_topic", rviz_topic, std::string(""));

  if (!rviz_topic.empty()) {
      rviz_plotter_ = new GraspPlotter(node, grasp_detector_->getHandSearchParameters().hand_geometry_);
      use_rviz_ = true;
    }
  else {
      use_rviz_ = false;
  }

  std::string cloud_topic;
  node.param("cloud_topic", cloud_topic, std::string("/camera/depth_registered/points"));
  cloud_sub_ = node.subscribe(cloud_topic, 1, &GraspDetectionServerPRL::cloud_callback, this);

  node.getParam("workspace", workspace_);

}


bool GraspDetectionServerPRL::detectGrasps(gpd_ros::detect_grasps_prl::Request& req, gpd_ros::detect_grasps_prl::Response& res){

  ROS_INFO("Received service request ...");


    if (has_cloud_){
      // Detect grasps in point cloud.
      std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = detectGraspPoses();

      // Reset the system.
      has_cloud_ = false;
      ROS_INFO("Waiting for point cloud to arrive ...");

      if (grasps.size() > 0){
        ROS_INFO("VISUALIZING");
          // Visualize the detected grasps in rviz.
          if (use_rviz_){
              rviz_plotter_->drawGrasps(grasps, frame_);
          }

          // Publish the detected grasps.
          gpd_ros::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps, cloud_camera_header_);
          res.grasp_configs = selected_grasps_msg;
          ROS_INFO_STREAM("Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
          return true;
     }
    }

    ROS_WARN("No grasps detected!");
    return false;

}


void GraspDetectionServerPRL::cloud_callback(const sensor_msgs::PointCloud2& msg){

  if (!has_cloud_){
    delete cloud_camera_;
    cloud_camera_ = NULL;
  }

  Eigen::Matrix3Xd view_points(3,1);
  view_points.col(0) = view_point_;

  if (msg.fields.size() == 6 && msg.fields[3].name == "normal_x" && msg.fields[4].name == "normal_y"
      && msg.fields[5].name == "normal_z"){
    PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
    pcl::fromROSMsg(msg, *cloud);
    cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);
    cloud_camera_header_ = msg.header;
    // ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points and normals.");
  }
  else{
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(msg, *cloud);
    cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);
    cloud_camera_header_ = msg.header;
    ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");
  }

  has_cloud_ = true;
  frame_ = msg.header.frame_id;
}

std::vector<std::unique_ptr<gpd::candidate::Hand>> GraspDetectionServerPRL::detectGraspPoses(){
  // detect grasp poses
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;

  // preprocess the point cloud
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  // detect grasps in the point cloud
  grasps = grasp_detector_->detectGrasps(*cloud_camera_);

  return grasps;
}


int main(int argc, char** argv){
  // seed the random number generator
  std::srand(std::time(0));

  // initialize ROS
  ros::init(argc, argv, "detect_grasps_server_prl");
  ros::NodeHandle node("~");

  GraspDetectionServerPRL grasp_detection_server_prl(node);

  ros::ServiceServer service = node.advertiseService("detect_grasps", &GraspDetectionServerPRL::detectGrasps,
                                                     &grasp_detection_server_prl);

  ROS_INFO("Ready to detect Grasps");
  ros::spin();

  return 0;
}
