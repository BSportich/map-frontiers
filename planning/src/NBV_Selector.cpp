#include <Eigen/Eigen>
#include <iostream>
#include "planning/modules/ViewGenerator.h"
#include "planning/modules/ViewEvaluator.h"
#include "planning/data/type_conversions.h"
#include "planning/data/visualization_marker.h"
#include <string>
#include <math.h> 
#include <chrono>
#include <random>
#include <numeric>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include "voxblox/utils/timing.h"
#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ptcloud_vis.h"
#include <voxblox_map/voxblox_map.h>
#include <voxblox_msgs/Layer.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>


struct system_parameters
{

    ///////////////NAVIGATION
    float tolerance_distance ; 
    double threshold_known ;

    //////////////View Generation parameters
    std::string method_view_generation ; 
    float distance_min ;
    float distance_max ;
    int subsampling_views ;
    float robot_radius ; 
    int radius_surface_max ; 
    float angle_low ;
    float angle_high ;
    ///////////////

    //////////////View Evaluator parameters
    float value_frontier ;
    ///////////////

    //////////////LIDAR Parameters
    Eigen::Vector3d mounting_translation_;  // x,y,z [m]
    Eigen::Quaterniond mounting_rotation_;  // x,y,z,w quaternion
    //sensor parameters
    double p_ray_length ;  // params for camera model
    double p_fov_x ;  // Total fields of view [deg], expected symmetric w.r.t.
    // sensor facing direction
    double p_fov_y ;
    int p_resolution_x ;
    int p_resolution_y ; // high number attendu
    double p_sampling_time;
};


class NBV_Selector
{
private:
    /* data */
    voxblox_map::VoxbloxMap m_map;
    std::string world_frame_;

    ViewGenerator m_view_generator;
    std::vector<ViewCandidate> views;
    std::vector<Eigen::Vector3d> frontiers_set ;
    std::vector<Eigen::Vector3d> frontiers_subset ;

    ViewEvaluator m_view_evaluator;
    SensorModel m_sensor_model;
    ros::Time last_nbv_ ;

    //frontiers pointclouds
    pcl::PointCloud<pcl::PointXYZRGB> frontiers_pointcloud ;
    pcl::PointCloud<pcl::PointXYZRGB> frontiers_sub_pointcloud ;
    pcl::PointCloud<pcl::PointXYZRGB> values_for_eval_pointcloud ; 
    pcl::PointCloud<pcl::PointXYZRGB> values_for_eval_pointcloud2 ; 

    //views generated poses
    geometry_msgs::PoseArray views_set; 
    int m_sub_sample_size_ ;
    float m_tolerance_distance_ ; 
    double m_threshold_known ; 

    bool m_surface_frontiers;
    ViewCandidate m_current_goal;
    ViewCandidate m_current_pos;
    bool m_availability;
    bool m_is_idle;

    //ROS COMMUNICATION
    ros::NodeHandle n;
    ros::Subscriber sub_map_tsdf;
    ros::Subscriber sub_map_esdf;
    ros::Subscriber sub_pos;
    ros::Subscriber sub_idle;
    ros::Publisher pub_goal;
    ros::ServiceServer start_server;
    ros::ServiceServer stop_server;
    //frontiers and tsdfs
    ros::Publisher pub_pointcloud;
    ros::Publisher pub_frontiers;
    ros::Publisher pub_sub_frontiers;
    ros::Publisher pub_test_values;
    ros::Publisher pub_test_values2;
    //views and selected views
    ros::Publisher pub_views ; 
    ros::Publisher pub_views_marker ; 
    ros::Publisher pub_nbv ; 


    //MAP PARAMETERS
    float z_max;
    float z_min;
    float y_max;
    float y_min;
    float x_max;
    float x_min;

    //TEAM AND COORDINATIONS ANALYSIS
    int m_team_size;
    int m_team_id;
    std::vector<int> m_team;
    std::unordered_map<int, ViewCandidate> m_team_pos;

    //STATES
    const static unsigned char AVAILABLE = 0;  // NOLINT
    const static unsigned char BUSY = 1;      // NOLINT

    // GENERAL BEHAVIOUR
    bool timer_ = false;
    bool verbose_ = false;
    bool extra_viz_ = false;
    bool is_started_ = false;

public:
    NBV_Selector();
    NBV_Selector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, int team_id, std::vector<int> robot_team, system_parameters sys_param);
    void updateFrontiers();
    void sample_subset_frontiers();
    void sample_subset_frontiers_discrete();
    void sample_subset_frontiers_shells();


    bool isFrontierVoxel_ESDF(const Eigen::Vector3d& voxel);

    bool isInBoundingBox(const Eigen::Vector3d& voxel);

    void publishAllUpdatedTsdfVoxels() ;
    void publish_all_frontiers();
    void publish_sub_frontiers();

    void generate_views();
    void publish_views();
    void publish_goal();

    void select_next_best_view(); 

    //Tests functions
    //void test_publish();

    //callbacks
    void tSDFCallback(const voxblox_msgs::Layer& layer_msg);
    void eSDFCallback(const voxblox_msgs::Layer& layer_msg);
    void posCallback(const nav_msgs::Odometry& msg_odom);
    void isIdleCallback(const std_msgs::Bool& msg_is_idle);
    bool startCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT
    bool stopCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT
  

    ~NBV_Selector();
};

NBV_Selector::NBV_Selector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, int team_id, std::vector<int> robot_team, system_parameters sys_param)
{
    n = nh;
    //initialization
    m_team_id = team_id ;
    m_team_size = robot_team.size() ;
    m_team = robot_team;
    // m_team_pos();
    m_availability = AVAILABLE;
    m_is_idle = true;
    last_nbv_ = ros::Time::now();

    // Get ros params
    //taken for default value in the code of tsdf_map.h and esdf_map.h
    double voxel_size = 0.2;  // in m
    int voxels_per_side = 16;

    nh_private.param("voxel_size", voxel_size, voxel_size);
    ROS_INFO("Received voxel_size: %f found", voxel_size);

    nh_private.param("voxels_per_side", voxels_per_side, voxels_per_side);
    ROS_INFO("Received voxels_per_side: %i found", voxels_per_side);

    nh_private.param("sub_sample_size", sys_param.subsampling_views, sys_param.subsampling_views);
    ROS_INFO("Received sub_sample_size: %i", sys_param.subsampling_views);

    nh_private.param("timer", timer_, timer_);
    ROS_INFO("Enabling timer: %s", timer_ ? "true" : "false");

    nh_private.param("verbose", verbose_, verbose_);
    ROS_INFO("Enabling verbose: %s", verbose_ ? "true" : "false");

    nh_private.param("extra_viz", extra_viz_, extra_viz_);
    ROS_INFO("Enabling extra_viz: %s", extra_viz_ ? "true" : "false");

    world_frame_ = "world";
    //map
    m_map = voxblox_map::VoxbloxMap(voxel_size, voxels_per_side);

    //modules
    //m_view_generator.set_map(m_map);
    std::string method = sys_param.method_view_generation ;
    m_view_generator = ViewGenerator(method, sys_param.distance_min, sys_param.distance_max, m_map, sys_param.robot_radius, sys_param.angle_low, sys_param.angle_high);
    m_sensor_model = SensorModel( sys_param.p_ray_length, sys_param.p_fov_x, sys_param.p_fov_y, sys_param.p_resolution_x, sys_param.p_resolution_y, sys_param.p_sampling_time);
    m_view_evaluator = ViewEvaluator(m_map, "", m_sensor_model, sys_param.threshold_known, sys_param.radius_surface_max );
    m_sub_sample_size_ = sys_param.subsampling_views ; 
    m_tolerance_distance_ = sys_param.tolerance_distance ; 
    m_threshold_known = sys_param.threshold_known ; 

    //frontiers
    frontiers_set = std::vector<Eigen::Vector3d>();
    frontiers_subset = std::vector<Eigen::Vector3d>();
    frontiers_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    frontiers_sub_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    values_for_eval_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    values_for_eval_pointcloud2 = pcl::PointCloud<pcl::PointXYZRGB>(); 

    //views
    views = std::vector<ViewCandidate>();
    views_set = geometry_msgs::PoseArray();


    //ros initialization
    // ros::init(argc, argv, "NBV_selector_node robot ");
    sub_map_tsdf = n.subscribe("tsdf_map_out", 10, &NBV_Selector::tSDFCallback, this);
    sub_map_esdf = n.subscribe("esdf_map_out", 10, &NBV_Selector::eSDFCallback, this);
    sub_pos = n.subscribe("groundtruth/odom", 20, &NBV_Selector::posCallback, this);
    sub_idle = n.subscribe("is_idle", 20, &NBV_Selector::isIdleCallback, this);
    pub_goal = n.advertise<geometry_msgs::PoseStamped>("pos_goal", 20); //to redefine msg type
    pub_pointcloud = n.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "test_point_cloud", 1, true);
    pub_frontiers = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "frontiers_point_cloud", 1, true);
    pub_sub_frontiers = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "subfrontiers_point_cloud", 1, true);

    pub_test_values = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "empty_point_cloud", 1, true);
    
    pub_test_values2 = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "unknown_point_cloud", 1, true);

    pub_views = n.advertise<geometry_msgs::PoseArray>("views", 1, true);
    pub_views_marker = n.advertise<visualization_msgs::MarkerArray>("views_marker", 1, true);
    pub_nbv = n.advertise<geometry_msgs::Pose>("the_next_best_view", 1, true);

    start_server = n.advertiseService("start_NBV_selector", &NBV_Selector::startCallback, this);
    stop_server = n.advertiseService("stop_NBV_selector", &NBV_Selector::stopCallback, this);


    //ros::spin()
}

// bool NBV_Selector::isFrontierVoxel_ESDF(const Eigen::Vector3d& voxel){
//   unsigned char voxel_state;
//   if ( m_frontier6 ) {
//     for (int i = 0; i < 6; ++i) {
//       voxel_state = m_map.getVoxelState_ESDF(voxel + c_neighbor_voxels_[i]);
//       if (voxel_state == voxblox_map::VoxbloxMap::UNKNOWN) {
//         continue;
//       }
//       if (m_surface_frontiers ) {
//         return voxel_state == voxblox_map::VoxbloxMap::OCCUPIED;
//       } else {
//         return true;
//       }
//     }
//   } else {
//     for (int i = 0; i < 26; ++i) {
//       voxel_state = m_map.getVoxelState_ESDF(voxel + c_neighbor_voxels_[i]);
//       if (voxel_state == voxblox_map::VoxbloxMap::UNKNOWN) {
//         continue;
//       }
//       if ( m_surface_frontiers ) {
//         return voxel_state == voxblox_map::VoxbloxMap::OCCUPIED;
//       } else {
//         return true;
//       }
//     }
//   }
//   return false;

// }


void NBV_Selector::updateFrontiers(){
    ros::Time start_update_frontiers = ros::Time::now();

    unsigned char current_state;
    ROS_INFO_COND(verbose_, "Updated frontiers: %lu found", frontiers_set.size());

    voxblox::BlockIndexList blocks;
    m_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);
    frontiers_pointcloud.clear();
    values_for_eval_pointcloud.clear();
    values_for_eval_pointcloud2.clear();
    frontiers_set.clear();

    // Cache layer settings.
    size_t vps = m_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    for (const voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel>& block = m_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->getBlockByIndex(index);

      voxblox::Point origin = block.origin();

      for (size_t linear_index = 0; linear_index < num_voxels_per_block;
          ++linear_index) {
        voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
        const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
        Eigen::Vector3d coord_3d = Eigen::Vector3d(coord.x(), coord.y(), coord.z());

        //if ( m_view_evaluator.isFrontierVoxel_TSDF(coord_3d, m_threshold_known)){
        if ( m_view_evaluator.isSurfaceFrontier_TSDF(coord_3d) ){
          frontiers_set.push_back( coord_3d );

          pcl::PointXYZRGB point;
          point.x = coord.x();
          point.y = coord.y();
          point.z = coord.z();
          point.r = 0;
          point.g = 0;
          point.b = 0;
          frontiers_pointcloud.push_back(point);
        }

        //empty pointcloud
        current_state = m_map.getVoxelState_TSDF(coord_3d, m_threshold_known);
        if ( current_state == voxblox_map::VoxbloxMap::FREE ){

          pcl::PointXYZRGB point;
          point.x = coord.x();
          point.y = coord.y();
          point.z = coord.z();
          point.r = 0;
          point.g = 0;
          point.b = 0;
          values_for_eval_pointcloud.push_back(point);
        }

        //unknown pointcloud
        if ( current_state == voxblox_map::VoxbloxMap::UNKNOWN ){

          pcl::PointXYZRGB point;
          point.x = coord.x();
          point.y = coord.y();
          point.z = coord.z();
          point.r = 0;
          point.g = 0;
          point.b = 0;
          values_for_eval_pointcloud2.push_back(point);
        }

      }

    //block.voxel_size()
    }
    ROS_INFO_ONCE("Frontiers updated!");
    ros::Time end_update_frontiers = ros::Time::now();
    ros::Duration duration = end_update_frontiers - start_update_frontiers;
    ROS_INFO_COND(timer_, "[NBV_Selector][updateFrontiers] %.4f s", duration.toSec());
    

 }


void NBV_Selector::sample_subset_frontiers(){
  ros::Time start_sample_subset_frontiers = ros::Time::now();
  frontiers_sub_pointcloud.clear();
  frontiers_subset.clear();

  std::vector<float> distances_table(frontiers_set.size());

  if( frontiers_set.size() > m_sub_sample_size_ ){

    double min_value_distance = std::numeric_limits<double>::max() ; 
    double max_value_distance = std::numeric_limits<double>::min() ; 
    double total_distance = 0 ;

    for(int i=0; i< frontiers_set.size(); i++){
      
      Eigen::Vector3d current_pos = Eigen::Vector3d( m_current_pos.x, m_current_pos.y, m_current_pos.z );
      double distance_frontier = (frontiers_set[i] - current_pos).norm();
      distances_table[i] = distance_frontier ; 

      if(distance_frontier > max_value_distance){
        max_value_distance = distance_frontier;
      }
      if(distance_frontier < min_value_distance){
        min_value_distance = distance_frontier;
      }

    }

    float threshold_tirage = m_sub_sample_size_ / frontiers_set.size() ;
    float value_tirage = -1 ; 
    int i = 0 ; 
    while((frontiers_subset.size() < m_sub_sample_size_) && (i < frontiers_set.size() )){

        value_tirage = (static_cast<float>(rand()) / RAND_MAX) ; 
        threshold_tirage = m_sub_sample_size_ / frontiers_set.size() ;
        threshold_tirage = threshold_tirage *  ( (max_value_distance - distances_table[i] ) / (max_value_distance - min_value_distance )); 
        
        ROS_INFO_COND(verbose_, "[Sampling] in the while loop ... %d", i);
        ROS_INFO_COND(verbose_, "[Sampling] in the while loop ... %d", frontiers_subset.size());
        if(value_tirage > threshold_tirage){

          frontiers_subset.push_back(frontiers_set[i]);

          pcl::PointXYZRGB point;
          point.x = frontiers_set[i].x();
          point.y = frontiers_set[i].y();
          point.z = frontiers_set[i].z();
          point.r = 0;
          point.g = 0;
          point.b = 0;
          frontiers_sub_pointcloud.push_back(point);
          

        }
        i=i+1;
        ROS_INFO_COND(verbose_, "[Sampling] End while loop");
    }
    


  }
  else {

    frontiers_subset = frontiers_set ; //careful ! Seems like a deep copy but not sure
  }

  ros::Time end_sample_subset_frontiers = ros::Time::now();
  ros::Duration duration = end_sample_subset_frontiers - start_sample_subset_frontiers;
  ROS_INFO_COND(timer_, "[NBV_Selector][sample_subset_frontiers] %.4f s", duration.toSec());
}

void NBV_Selector::sample_subset_frontiers_shells(){
  ROS_INFO_COND(verbose_, "[Sampling] shells start");
  float value_tirage = (static_cast<float>(rand()) / RAND_MAX) ; 
  int index_id = -1 ; 
  for(int i =0 ; i < m_sub_sample_size_; i++ ){

    value_tirage = (static_cast<float>(rand()) / RAND_MAX) * m_sub_sample_size_ ; 
    index_id = static_cast<int>(value_tirage) ;
    frontiers_subset.push_back( frontiers_set[index_id] ) ; 


  }

  ROS_INFO_COND(verbose_, "[Sampling] shells end");

}

void NBV_Selector::sample_subset_frontiers_discrete(){
  ros::Time start_sample_subset_frontiers = ros::Time::now();

  frontiers_sub_pointcloud.clear();
  frontiers_subset.clear();

  std::vector<float> distances_table(frontiers_set.size());
  std::vector<float> weight_table(frontiers_set.size());

  if( frontiers_set.size() > m_sub_sample_size_ ){

    double min_value_distance = std::numeric_limits<double>::max() ; 
    double max_value_distance = std::numeric_limits<double>::min() ; 
    double total_distance = 0 ;

    for(int i=0; i< frontiers_set.size(); i++){
      
      Eigen::Vector3d current_pos = Eigen::Vector3d( m_current_pos.x, m_current_pos.y, m_current_pos.z );
      double distance_frontier = (frontiers_set[i] - current_pos).norm();
      distances_table[i] = distance_frontier ; 
      weight_table[i] = 1/( distance_frontier + 1e-6);

      if(distance_frontier > max_value_distance){
        max_value_distance = distance_frontier;
      }
      if(distance_frontier < min_value_distance){
        min_value_distance = distance_frontier;
      }

    }
    ROS_INFO_COND(verbose_, "[Sampling][Before generating distribution] ");

    float sum_weight = std::accumulate( weight_table.begin(), weight_table.end(), 0); // sum of weights
    for(auto& w : weight_table){ w = w / sum_weight; } //normalize weights

    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> dist(weight_table.begin(), weight_table.end());

    ROS_INFO_COND(verbose_, "[Sampling][After generating distribution] ");

    for(int i =0; i < m_sub_sample_size_ ; i++){

          int idx = dist(gen);

          frontiers_subset.push_back(frontiers_set[idx]);

          pcl::PointXYZRGB point;
          point.x = frontiers_set[idx].x();
          point.y = frontiers_set[idx].y();
          point.z = frontiers_set[idx].z();
          point.r = 0;
          point.g = 0;
          point.b = 0;
          frontiers_sub_pointcloud.push_back(point);
          

    }
        
    
    


  }
  else {

    frontiers_subset = frontiers_set ; 
  }


  ros::Time end_sample_subset_frontiers = ros::Time::now();
  ros::Duration duration = end_sample_subset_frontiers - start_sample_subset_frontiers;
  ROS_INFO_COND(timer_, "[NBV_Selector][sample_subset_frontiers_discrete] %.4f s", duration.toSec());


}

NBV_Selector::~NBV_Selector()
{
}


void NBV_Selector::publishAllUpdatedTsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud_d;
  createDistancePointcloudFromTsdfLayer(
      *m_map.get_tsdf_map_pointer()->getTsdfLayerPtr(), &pointcloud_d);
  pointcloud_d.header.frame_id = world_frame_;
  pub_pointcloud.publish(pointcloud_d);

  // // Create a pointcloud with gradient direction = intensity.
  // pcl::PointCloud<pcl::PointXYZI> pointcloud_g;
  // createGradientPointcloudFromTsdfLayer(
  //     m_map.get_tsdf_map_pointer()->getTsdfLayerPtr(), &pointcloud_g);
  // pointcloud_g.header.frame_id = world_frame_;
  // gsdf_pointcloud_pub_.publish(pointcloud_g);
}

void NBV_Selector::publish_all_frontiers(){
  frontiers_pointcloud.header.frame_id = world_frame_;
  values_for_eval_pointcloud.header.frame_id = world_frame_;
  values_for_eval_pointcloud2.header.frame_id = world_frame_;
  pub_frontiers.publish(frontiers_pointcloud);
  pub_test_values.publish( values_for_eval_pointcloud);
  pub_test_values2.publish( values_for_eval_pointcloud2);

}

void NBV_Selector::publish_sub_frontiers(){
  frontiers_sub_pointcloud.header.frame_id = world_frame_;
  pub_sub_frontiers.publish(frontiers_sub_pointcloud);

  //Analyse ESDF
}

// void NBV_Selector::publish_analysis_map(){

//   values_for_eval_pointcloud.header.frame_id = world_frame_;
//   values_for_eval_pointcloud2.header.frame_id = world_frame_;
//   pub_test_values.publish( values_for_eval_pointcloud);
//   pub_test_values2.publish( values_for_eval_pointcloud2);
// }

void NBV_Selector::tSDFCallback(const voxblox_msgs::Layer& layer_msg){
  ROS_INFO_COND(verbose_, "[TSDF callback] Entering TSDF callback");
  if (!is_started_)
    return;
  ROS_INFO_COND(verbose_, "[TSDF callback] Activated "); 

  voxblox::timing::Timer receive_map_timer("map/receive_tsdf");

  bool success =
      voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(layer_msg, m_map.get_tsdf_map_pointer()->getTsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "MAP FRONTIERS : Got an invalid TSDF map message!");
    LOG(ERROR) << "layer_msg voxel size = " << layer_msg.voxel_size << " map layer voxel size = " << m_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->voxel_size();
    LOG(ERROR) << "layer_msg voxel per side = " << layer_msg.voxels_per_side << " map layer voxel per side = " << m_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->voxels_per_side();
  } else {
    ROS_INFO_COND(verbose_, "[TSDF callback] Got an TSDF map from ROS topic!");
    // publishAllUpdatedTsdfVoxels();
    // ROS_INFO_COND(verbose_, "[TSDF callback] Published pointclouds");

    ROS_INFO_COND(verbose_, "[TSDF callback] THERE ARE %d FRONTIERS", frontiers_set.size() );
    //sample frontiers
    //sample_subset_frontiers_shells();
    int sample_size = std::min(m_sub_sample_size_, static_cast<int>(frontiers_pointcloud.size()));
    m_sub_sample_size_ = 100; 
    if(frontiers_set.size() > m_sub_sample_size_){
       frontiers_subset.assign(frontiers_set.end() - m_sub_sample_size_ , frontiers_set.end() );

       // Ensure we don't exceed the original point cloud size

      // Clear the destination point cloud
      frontiers_sub_pointcloud.clear();
      // Copy the first 'sample_size' points
      for (int i = 0; i < sample_size; i++) {
          frontiers_sub_pointcloud.push_back(frontiers_pointcloud.points[i]);
      }

       ROS_INFO_COND(verbose_, "[TSDF callback] SAMPLING");
     }
    
    
    publish_sub_frontiers();
    ROS_INFO_COND(verbose_, "[TSDF callback] PUBLISHING SUB FRONTIERS");

    //SEND PROCEDURE
    voxblox_msgs::Layer layer_msg;



    }
  


}

void NBV_Selector::eSDFCallback(const voxblox_msgs::Layer& layer_msg){
  ROS_INFO_COND(verbose_, "[ESDF callback] Entering ESDF callback");
  if (!is_started_)
    return;
  ROS_INFO_COND(verbose_, "[ESDF callback] Activated");
  voxblox::timing::Timer receive_map_timer("map/receive_esdf");

  bool success =
      voxblox::deserializeMsgToLayer<voxblox::EsdfVoxel>(layer_msg, m_map.get_esdf_map_pointer()->getEsdfLayerPtr());
  ROS_INFO_COND(verbose_, "[ESDF callback] Deserialized done ! ");

  if (!success) {
    ROS_ERROR_THROTTLE(10, "MAP FRONTIERS : Got an invalid ESDF map message!");
    ROS_INFO_COND(verbose_, "[ESDF callback] Deserialized failed ! ");
  } else {
    ROS_INFO_COND(verbose_, "[ESDF callback] Deserialized sucess ! ");
    updateFrontiers();
    ROS_INFO_COND(verbose_, "[ESDF callback] Updated frontiers ! ");
    if (extra_viz_)
      publish_all_frontiers();
    ROS_INFO_COND(verbose_, "Frontiers published !");
    generate_views() ; 
    ROS_INFO_COND(verbose_, "Views generated !" );
    publish_views();
    ROS_INFO_COND(verbose_,"Views published !");

    }
  


}

void NBV_Selector::posCallback(const nav_msgs::Odometry& msg_odom){ // TO FIX : Use tf/odometry in the multi robot case
  if (!is_started_)
      return;

  

  ROS_INFO_COND(verbose_, "POS CALLBACK");
  m_current_pos.o_x = m_current_pos.x ; 
  m_current_pos.o_y = m_current_pos.y;
  m_current_pos.o_z = m_current_pos.z;

  m_current_pos.x = msg_odom.pose.pose.position.x ;
  m_current_pos.y = msg_odom.pose.pose.position.y ;
  m_current_pos.z = msg_odom.pose.pose.position.z ;
  m_current_pos.q_x = msg_odom.pose.pose.orientation.x ;
  m_current_pos.q_y = msg_odom.pose.pose.orientation.y ;
  m_current_pos.q_z = msg_odom.pose.pose.orientation.z ;
  m_current_pos.q_w = msg_odom.pose.pose.orientation.w ;

  if( m_availability == BUSY){ //TO DO : ADD ORIENTATION
    float pos_test = pow((m_current_pos.x - m_current_goal.x ), 2)   + pow((m_current_pos.y - m_current_goal.y ),2) + pow((m_current_pos.z - m_current_goal.z ),2) ; 
    if(pos_test < m_tolerance_distance_ ){
      m_availability = AVAILABLE ; 
      ROS_INFO_COND(verbose_, "ROBOT IS NOW AVAILABLE");

    }

  }


  ros::Duration duration = ros::Time::now() - last_nbv_ ; 
  // if the robot has been on a standstill for more than one sec, we allow it to have a new goal
  if( m_is_idle &&  ( duration.toSec() > 1.0 ) ){
    m_availability = AVAILABLE ; 
  }

  //if the robot hasn't had a new goal in the last second and is available, find nbv
  if( (m_availability == AVAILABLE) && ( duration.toSec() > 1.0 ) ) {

    select_next_best_view();
    last_nbv_ = ros::Time::now();

    publish_goal();
    m_availability = BUSY ; 
    ROS_INFO_COND(verbose_, "ROBOT IS NOW BUSY");


    
  }
  else {
    ROS_INFO_COND(verbose_, "ROBOT IS NOT AVAILABLE %f ", duration.toSec()  );
  }


}

void NBV_Selector::isIdleCallback(const std_msgs::Bool& msg_is_idle) {
  if (verbose_ && m_is_idle != msg_is_idle.data) {
    if (m_is_idle)
      ROS_INFO("drone went from %s to %s", "idle", "moving");
    else
      ROS_INFO("drone went from %s to %s", "moving", "idle");
  }
  m_is_idle = msg_is_idle.data;
}

bool NBV_Selector::startCallback(
    std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response&
    /*response*/) {  // NOLINT
  ROS_INFO("Received a start service call.");
  is_started_ = true;
  return true;  // Return true to indicate the callback handled the response
}

bool NBV_Selector::stopCallback(
    std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response&
    /*response*/) {  // NOLINT
  ROS_INFO("Received a stop service call.");
  is_started_ = false;
  return true;  // Return true to indicate the callback handled the response
}

void NBV_Selector::publish_goal(){ 

  geometry_msgs::PoseStamped next_goal ;

  next_goal.header.stamp = ros::Time::now();  // Set timestamp
  next_goal.header.frame_id = world_frame_;
  map_frontiers::conversions::ViewCandidateToGeometryPose(m_current_goal, next_goal.pose);
  pub_goal.publish(next_goal);

}

void NBV_Selector::generate_views(){
  ros::Time start_generate_views = ros::Time::now();
  //m_view_generator.generateViews(frontiers_set);
  m_view_generator.generateViews(frontiers_subset);
  views = m_view_generator.getViewCandidates();
  ros::Time end_generate_views = ros::Time::now();
  ros::Duration duration = end_generate_views - start_generate_views;
  ROS_INFO_COND(timer_, "[NBV_Selector][generate_views] %.4f s", duration.toSec());
}

void NBV_Selector::publish_views(){
  if (!extra_viz_)
    return;
  visualization_msgs::MarkerArray marker_array;
  float range_for_viz = 0.3;

  views_set.poses.clear();
  for(int i = 0; i < views.size(); i++){
    geometry_msgs::Pose temp_view;
    map_frontiers::conversions::ViewCandidateToGeometryPose(views[i], temp_view);
    geometry_msgs::Point temp_frontier;
    temp_frontier.x = views[i].o_x;
    temp_frontier.y = views[i].o_y;
    temp_frontier.z = views[i].o_z;
    views_set.poses.push_back(temp_view);

    // Create markers to represent the fov the view would have
    marker_array.markers.push_back(map_frontiers::visualization::CreateHorizontalFOVMarker(temp_view, i, range_for_viz));
    marker_array.markers.push_back(map_frontiers::visualization::CreateVerticalFOVMarker(temp_view, i, range_for_viz));
    marker_array.markers.push_back(map_frontiers::visualization::CreateViewToFrontiersLine(temp_view, temp_frontier, i));
  }
  views_set.header.frame_id = world_frame_;
  pub_views.publish(views_set);
  pub_views_marker.publish(marker_array);
}


//Called in poscallback after pose updated
void NBV_Selector::select_next_best_view(){
  
  ROS_INFO_COND(verbose_, "SELECTING VIEWS NOW");
  
  // ROS_INFO_COND(verbose_, "EMPTY SPACE");

  //evaluate views
  std::vector<float> values_views;
  int index_of_nbv = -1 ;
  float max_value_nbv = -1 ; 
  float temp_value = -1 ; 
  m_current_goal = m_current_pos;
  float temp_value_angular = 0 ;
  ROS_INFO_COND(verbose_, "EXAMINING %d", views.size());
  // for(int i=0;i< views.size();i++){

  //   ROS_INFO_COND(verbose_, "GET VOXELS VIEW %d", i);


  //   ViewCandidate view = views[i] ; 
  //   std::vector<Eigen::Vector3d> visible_voxels; 
  //   Eigen::Vector3d pos = Eigen::Vector3d( view.x, view.y, view.z);
  //   Eigen::Quaterniond orient = Eigen::Quaterniond( view.q_x, view.q_y, view.q_z, view.q_w);

  //   ros::Time start_get_visible_voxels_lidar = ros::Time::now();
  //   m_view_evaluator.getVisibleVoxels_LIDAR(
  //   &visible_voxels, pos, orient) ;
  //   ros::Time end_get_visible_voxels_lidar = ros::Time::now();
  //   ros::Duration duration = end_get_visible_voxels_lidar - start_get_visible_voxels_lidar;
  //   ROS_INFO_COND(timer_, "[NBV_Selector][getVisibleVoxels_LIDAR] %.4f s", duration.toSec());

  //   ROS_INFO_COND(verbose_, "COUNTING FRONTIERS VIEW %d", i);
    
  //   ros::Time start_view_evaluation = ros::Time::now();
  //   temp_value = m_view_evaluator.count_frontiers_view(visible_voxels);
  //   //temp_value = m_view_evaluator.evaluate_view_image(visible_voxels);
  //   ros::Time end_view_evaluation = ros::Time::now();
  //   duration = end_view_evaluation - start_view_evaluation;
  //   ROS_INFO_COND(timer_, "[NBV_Selector][Evaluate View] %.4f s", duration.toSec());
  //   Eigen::Vector3d current_pos_vector( m_current_pos.x ,m_current_pos.y , m_current_pos.z );
  //   temp_value_angular = m_view_evaluator.evaluate_view_angular( view, current_pos_vector ) ; 
  //   values_views.push_back(temp_value);
  //   if( temp_value > max_value_nbv){
  //     index_of_nbv = i;
  //     max_value_nbv = temp_value;

  //   }

    //break ; 


  // }


  // //select views

  // if (views.size() > 0){
  //   ROS_INFO_COND(verbose_, "UPDATING CURRENT GOAL ");
  //   m_current_goal = views[index_of_nbv];
  // }




}


int main(int argc, char** argv) {
    ros::init(argc, argv, "nbv_selector_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");  

    system_parameters sys_params ; 
    /////////////// NAVIGATION
    sys_params.tolerance_distance = 0.2 ; //TO DO : CHECK UNITE

    sys_params.threshold_known = 0.0 ; //from Hardouin 0.3
    


    //////////////View Generation parameters
    sys_params.method_view_generation = "gradient" ; 
    sys_params.distance_min = 1;
    sys_params.distance_max = 2;
    sys_params.subsampling_views = 10 ;
    sys_params.robot_radius = 8; // max number of voxels occupied by the robots in one direction : if 5, robot is contained in a 5*5*5 voxel cube
    sys_params.angle_low = -25 ; // vertical angle below the drone 
    sys_params.angle_high = 57; // vertical angle above the drone 
    ///////////////

    //////////////View Evaluator parameters
    sys_params.value_frontier = 1 ;
    sys_params.radius_surface_max = 3 ; // zone around the frontier to search for surface voxels
    ///////////////

    //////////////LIDAR Parameters
    Eigen::Vector3d mounting_translation_;  // x,y,z [m]
    Eigen::Quaterniond mounting_rotation_;  // x,y,z,w quaternion
    //sensor parameters
    sys_params.p_ray_length = 10;  // params for camera model
    sys_params.p_fov_y = 360;  // Total fields of view [deg], expected symmetric w.r.t.
    // sensor facing direction
    sys_params.p_fov_x = 63.05;
    sys_params.p_resolution_x = 1000 ;
    sys_params.p_resolution_y = 1000; // high number attendu
    sys_params.p_sampling_time =1; 

    sys_params.p_fov_x *= M_PI / 180.0;
    sys_params.p_fov_y *= M_PI / 180.0;
    ///////////////


    ///////////////Robot team initialization
    int team_id = 1;
    std::vector<int> robot_team ; 
    robot_team.push_back(team_id);
    ///////////////

    NBV_Selector nbv_selector = NBV_Selector(nh, nh_private, team_id,  robot_team, sys_params);
    ros::spin();
    return 0;
}


