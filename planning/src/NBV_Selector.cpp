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

    //////////////Planning parameters
    float alpha;
    float beta ;
    bool gamma ; 
    bool occlusion;

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
    std::vector<ViewCandidate> views_history;
    std::vector<ViewCandidate> rejected_views;
    std::vector<Eigen::Vector3d> frontiers_set ;
    std::vector<Eigen::Vector3d> frontiers_subset ;

    ViewEvaluator m_view_evaluator;
    SensorModel m_sensor_model;
    ros::Time last_nbv_ ;

    //planning parameters
    float m_alpha ;
    float m_beta ; 
    bool m_gamma ;

    //frontiers pointclouds
    pcl::PointCloud<pcl::PointXYZRGB> frontiers_pointcloud ;
    pcl::PointCloud<pcl::PointXYZRGB> frontiers_sub_pointcloud ;
    pcl::PointCloud<pcl::PointXYZRGB> rejected_frontiers_pointcloud ;
    pcl::PointCloud<pcl::PointXYZRGB> values_for_eval_pointcloud ; 
    pcl::PointCloud<pcl::PointXYZRGB> values_for_eval_pointcloud2 ; 

    //views generated poses
    geometry_msgs::PoseArray views_set; 
    geometry_msgs::PoseArray rejected_views_set; 
    int m_sub_sample_size_ ;
    float m_tolerance_distance_ ; 
    double m_threshold_known ; 

    bool m_surface_frontiers;
    ViewCandidate m_current_goal;
    ViewCandidate m_current_pos;
    //Velocity angular or directional ?? 
    float m_vel_x ; 
    float m_vel_y ; 
    float m_vel_z ;

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
    ros::Publisher pub_rejected;
    ros::Publisher pub_test_values;
    ros::Publisher pub_test_values2;
    //views and selected views
    ros::Publisher pub_views ; 
    ros::Publisher pub_rejected_views ; 
    ros::Publisher pub_views_marker ; 
    ros::Publisher pub_views_rejected_marker ; 
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
    void publish_test_voxels();

    void generate_views();
    void publish_views();
    void publish_goal();

    void select_next_best_view(); 
    void next_best_view_closest_frontier();


    //Tests functions
    void visualize_voxels_ESDF();
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

    nh_private.param("p_ray_length", sys_param.p_ray_length, sys_param.p_ray_length);
    ROS_INFO("Received p_ray_length: %f", sys_param.p_ray_length);

    nh_private.param("sub_sample_size", sys_param.subsampling_views, sys_param.subsampling_views);
    ROS_INFO("Received sub_sample_size: %i", sys_param.subsampling_views);

    nh_private.param("alpha", sys_param.alpha, sys_param.alpha);
    ROS_INFO("Received image component coefficient: %f", sys_param.alpha);

    nh_private.param("beta", sys_param.beta, sys_param.beta);
    ROS_INFO("Received navigation component coefficient: %f", sys_param.beta);

    nh_private.param("gamma", sys_param.gamma, sys_param.gamma);
    if (sys_param.gamma) {
      ROS_INFO("Using angular and linear distances to compute the cost weight of each view.");
    } else {
      ROS_INFO("Using angular distance only to compute the cost weight of each view.");
    }

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
    m_view_evaluator = ViewEvaluator(m_map, "", m_sensor_model, sys_param.threshold_known, sys_param.radius_surface_max, sys_param.distance_min, sys_param.distance_max, sys_param.occlusion );
    m_sub_sample_size_ = sys_param.subsampling_views ; 
    m_tolerance_distance_ = sys_param.tolerance_distance ; 
    m_threshold_known = sys_param.threshold_known ; 

    m_alpha = sys_param.alpha ;
    m_beta = sys_param.beta ;
    m_gamma = sys_param.gamma ;


    //frontiers
    frontiers_set = std::vector<Eigen::Vector3d>();
    frontiers_subset = std::vector<Eigen::Vector3d>();
    frontiers_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    frontiers_sub_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    rejected_frontiers_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    values_for_eval_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    values_for_eval_pointcloud2 = pcl::PointCloud<pcl::PointXYZRGB>(); 

    //views
    views = std::vector<ViewCandidate>();
    views_history = std::vector<ViewCandidate>();
    views_set = geometry_msgs::PoseArray();
    rejected_views_set = geometry_msgs::PoseArray();


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

    pub_rejected =  n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
            "rejected_point_cloud", 1, true);

    pub_test_values = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "occupied_point_cloud", 1, true);
    
    pub_test_values2 = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "unknown_point_cloud", 1, true);

    pub_views = n.advertise<geometry_msgs::PoseArray>("views", 1, true);
    pub_rejected_views = n.advertise<geometry_msgs::PoseArray>("rejected_views", 1, true);
    pub_views_marker = n.advertise<visualization_msgs::MarkerArray>("views_marker", 1, true);
    pub_views_rejected_marker = n.advertise<visualization_msgs::MarkerArray>("views_marker_rejected", 1, true);
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
    frontiers_set.clear();
    values_for_eval_pointcloud.clear();
    values_for_eval_pointcloud2.clear();

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

        // ROS_INFO_COND(verbose_, "TESTING COORDINATES %f %f %f", coord.x(), coord.y(), coord.z());


        //if ( m_view_evaluator.isFrontierVoxel_TSDF(coord_3d, m_threshold_known)){
        Eigen::Vector3d unknown_vox_frontier;
        if ( m_view_evaluator.isSurfaceFrontier_TSDF(coord_3d, unknown_vox_frontier ) ){
          frontiers_set.push_back( coord_3d );

          // ROS_INFO_COND(verbose_, "Frontier found distance %f", m_map.getVoxelDistance_TSDF( coord_3d ));

          pcl::PointXYZRGB point;
          point.x = coord.x();
          point.y = coord.y();
          point.z = coord.z();
          point.r = 0;
          point.g = 0;
          point.b = 0;
          frontiers_pointcloud.push_back(point);

          // pcl::PointXYZRGB point2;
          // point2.x = unknown_vox_frontier.x();
          // point2.y = unknown_vox_frontier.y();
          // point2.z = unknown_vox_frontier.z();
          // point2.r = 255;
          // point2.g = 0;
          // point2.b = 0;
          // values_for_eval_pointcloud2.push_back(point2);


        }

        ///empty pointcloud
        // current_state = m_map.getVoxelState_TSDF(coord_3d, m_threshold_known);
        // if ( current_state == voxblox_map::VoxbloxMap::FREE ){

        //   pcl::PointXYZRGB point;
        //   point.x = coord.x();
        //   point.y = coord.y();
        //   point.z = coord.z();
        //   point.r = 0;
        //   point.g = 0;
        //   point.b = 0;
        //   values_for_eval_pointcloud.push_back(point);
        // }

        ///unknown pointcloud
        // if ( current_state == voxblox_map::VoxbloxMap::UNKNOWN ){

        //   pcl::PointXYZRGB point;
        //   point.x = coord.x();
        //   point.y = coord.y();
        //   point.z = coord.z();
        //   point.r = 0;
        //   point.g = 0;
        //   point.b = 0;
        //   values_for_eval_pointcloud2.push_back(point);
        // }

      }

    //block.voxel_size()
    }
    ROS_INFO_ONCE("Frontiers updated!");
    ros::Time end_update_frontiers = ros::Time::now();
    ros::Duration duration = end_update_frontiers - start_update_frontiers;
    ROS_INFO_COND(timer_, "[NBV_Selector][updateFrontiers] %.4f s", duration.toSec());
    

 }


 void NBV_Selector::visualize_voxels_ESDF(){
  ros::Time start_update_frontiers = ros::Time::now();

  unsigned char current_state;
  ROS_INFO_COND(verbose_, "Updated frontiers: %lu found", frontiers_set.size());

  voxblox::BlockIndexList blocks;
  m_map.get_esdf_map_pointer()->getEsdfLayerPtr()->getAllAllocatedBlocks(&blocks);
  values_for_eval_pointcloud.clear();
  values_for_eval_pointcloud2.clear();

  // Cache layer settings.
  size_t vps = m_map.get_esdf_map_pointer()->getEsdfLayerPtr()->voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  for (const voxblox::BlockIndex& index : blocks) {
  // Iterate over all voxels in said blocks.
  const voxblox::Block<voxblox::EsdfVoxel>& block = m_map.get_esdf_map_pointer()->getEsdfLayerPtr()->getBlockByIndex(index);

    voxblox::Point origin = block.origin();

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
        ++linear_index) {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const voxblox::EsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
      Eigen::Vector3d coord_3d = Eigen::Vector3d(coord.x(), coord.y(), coord.z());

      //empty pointcloud
      current_state = m_map.getVoxelState_ESDF(coord_3d);
      if ( current_state == voxblox_map::VoxbloxMap::OCCUPIED ){

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
  int sample_value = m_sub_sample_size_; 
  ROS_INFO("[Sampling] sampling value is %d", sample_value);
  if( m_sub_sample_size_ > frontiers_set.size()){
    sample_value = frontiers_set.size();
  }
  ROS_INFO("[Sampling] sampling value is %d", sample_value);

  ROS_INFO_COND(verbose_, "[Sampling] shells start");
  float value_tirage = (static_cast<float>(rand()) / RAND_MAX) ; 
  int index_id = -1 ; 
  frontiers_sub_pointcloud.clear();
  frontiers_subset.clear();

  for(int i =0 ; i < sample_value; i++ ){

    value_tirage = (static_cast<float>(rand()) / RAND_MAX) * (sample_value -1) ; 
    index_id = static_cast<int>(value_tirage) ;
    frontiers_subset.push_back( frontiers_set[index_id] ) ; 
    frontiers_sub_pointcloud.push_back( frontiers_pointcloud.points[index_id] );

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
  pub_frontiers.publish(frontiers_pointcloud);
}

void NBV_Selector::publish_test_voxels(){
  values_for_eval_pointcloud.header.frame_id = world_frame_;
  values_for_eval_pointcloud2.header.frame_id = world_frame_;
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

    updateFrontiers();
    ROS_INFO_COND(verbose_, "[TSDF callback] Updated frontiers ! ");

    if (extra_viz_){
      publish_all_frontiers();
      ROS_INFO_COND(verbose_, "Frontiers published !");
    }

    ROS_INFO_COND(verbose_, "[TSDF callback] THERE ARE %d FRONTIERS", frontiers_set.size() );

    // publish_test_voxels(); 
    ROS_INFO_COND(verbose_, "[TSDF callback] Published test voxels ! ");
    
    //sample frontiers
    sample_subset_frontiers_shells();


    ROS_INFO_COND(verbose_, "[TSDF callback] SAMPLING");
  
    
    
    publish_sub_frontiers();
    ROS_INFO_COND(verbose_, "[TSDF callback] PUBLISHING SUB FRONTIERS");
  }
    //SEND PROCEDURE
    //voxblox_msgs::Layer layer_msg;
  


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

    // visualize_voxels_ESDF();
    // //ROS_INFO_COND(verbose_, "[ESDF callback] Sorted ESDF voxels ! ");
    // publish_test_voxels(); 
    // //ROS_INFO_COND(verbose_, "[ESDF callback] Published test voxels ! ");

    int number_views = 0 ;
    int number_resampling = 0 ; 

    generate_views() ; 
    number_views = views.size();
    
    while( number_views == 0 && frontiers_set.size() > 0 ){
      number_resampling = number_resampling +1 ; 
      ROS_INFO_COND(verbose_, "[ESDF callback] Found no views... Resampling for the %d times ", number_resampling);
      //sample frontiers
      sample_subset_frontiers_shells();
      generate_views() ; 
      number_views = views.size();


    }

    ROS_INFO_COND(verbose_, "Views generated !" );
    publish_views();
    ROS_INFO_COND(verbose_,"Views published !");

    }
  


}

void NBV_Selector::posCallback(const nav_msgs::Odometry& msg_odom){ // TO FIX : Use tf/odometry in the multi robot case
  if (!is_started_)
      return;


  ///////////////// VERIFICATION OF MESSAGE INTEGRITY 

  if (std::isnan(msg_odom.twist.twist.linear.x)) {
      ROS_WARN("Warning: Odometry linear.x is NaN.");
  }
  if (std::isnan(msg_odom.twist.twist.linear.y)) {
      ROS_WARN("Warning: Odometry linear.y is NaN.");
  }
  if (std::isnan(msg_odom.twist.twist.linear.z)) {
      ROS_WARN("Warning: Odometry linear.z is NaN.");
  }

  // Check for NaN in twist.angular
  if (std::isnan(msg_odom.twist.twist.angular.x)) {
      ROS_WARN("Warning: Odometry angular.x is NaN.");
  }
  if (std::isnan(msg_odom.twist.twist.angular.y)) {
      ROS_WARN("Warning: Odometry angular.y is NaN.");
  }
  if (std::isnan(msg_odom.twist.twist.angular.z)) {
      ROS_WARN("Warning: Odometry angular.z is NaN.");
  }

  // Check for NaN in pose.position
  if (std::isnan(msg_odom.pose.pose.position.x)) {
      ROS_WARN("Warning: Odometry position.x is NaN.");
  }
  if (std::isnan(msg_odom.pose.pose.position.y)) {
      ROS_WARN("Warning: Odometry position.y is NaN.");
  }
  if (std::isnan(msg_odom.pose.pose.position.z)) {
      ROS_WARN("Warning: Odometry position.z is NaN.");
  }

  // Check for NaN in pose.orientation
  if (std::isnan(msg_odom.pose.pose.orientation.x)) {
      ROS_WARN("Warning: Odometry orientation.x is NaN.");
  }
  if (std::isnan(msg_odom.pose.pose.orientation.y)) {
      ROS_WARN("Warning: Odometry orientation.y is NaN.");
  }
  if (std::isnan(msg_odom.pose.pose.orientation.z)) {
      ROS_WARN("Warning: Odometry orientation.z is NaN.");
  }
  if (std::isnan(msg_odom.pose.pose.orientation.w)) {
      ROS_WARN("Warning: Odometry orientation.w is NaN.");
  }

  // Check if the message timestamp is zero
  if (msg_odom.header.stamp.isZero()) {
      ROS_WARN("Warning: Odometry message has no timestamp.");
  }

/////////////////////////////////////////////////////////////////


  ROS_INFO_COND(verbose_, "POS CALLBACK");
  m_current_pos.o_x = m_current_pos.x ; 
  m_current_pos.o_y = m_current_pos.y;
  m_current_pos.o_z = m_current_pos.z;
  // ROS_INFO_COND(verbose_, "CHECK 2 ");
  m_current_pos.x = msg_odom.pose.pose.position.x ;
  m_current_pos.y = msg_odom.pose.pose.position.y ;
  m_current_pos.z = msg_odom.pose.pose.position.z ;
  m_current_pos.q_x = msg_odom.pose.pose.orientation.x ;
  m_current_pos.q_y = msg_odom.pose.pose.orientation.y ;
  m_current_pos.q_z = msg_odom.pose.pose.orientation.z ;
  m_current_pos.q_w = msg_odom.pose.pose.orientation.w ;
  
  // ROS_INFO_COND(verbose_, "CHECK 3 ");
  //linear speed : should be angular ? 
  m_vel_x = msg_odom.twist.twist.linear.x ;
  m_vel_y = msg_odom.twist.twist.linear.y ;
  m_vel_z = msg_odom.twist.twist.linear.z ;
  // ROS_INFO_COND(verbose_, "[First] VEL VALUES ARE %f %f %f", m_vel_x, m_vel_y, m_vel_z);
  // ROS_INFO_COND(verbose_, "[First] POS VALUES ARE %f %f %f", m_current_pos.x,  m_current_pos.y ,  m_current_pos.z);
  // ROS_INFO_COND(verbose_, "[First] OR VALUES ARE %f %f %f %f", m_current_pos.q_x,  m_current_pos.q_y ,  m_current_pos.q_z, m_current_pos.q_w );
  

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
    ROS_INFO_COND(verbose_, " Going into selection");

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
  // if (!extra_viz_)
  //   return;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::MarkerArray marker_array_rejects;
  float range_for_viz = 0.3;

  views_set.poses.clear();
  rejected_views_set.poses.clear();

  
  //rejected_views
  rejected_views = m_view_generator.getRejectedViews() ; 
  for(int i = 0; i < rejected_views.size(); i++){
    geometry_msgs::Pose temp_view;
    map_frontiers::conversions::ViewCandidateToGeometryPose(rejected_views[i], temp_view);
    geometry_msgs::Point temp_frontier;
    temp_frontier.x = rejected_views[i].o_x;
    temp_frontier.y = rejected_views[i].o_y;
    temp_frontier.z = rejected_views[i].o_z;
    rejected_views_set.poses.push_back(temp_view);

    // Create markers to represent the fov the view would have
    marker_array_rejects.markers.push_back(map_frontiers::visualization::CreateHorizontalFOVMarker(temp_view, i, range_for_viz));
    marker_array_rejects.markers.push_back(map_frontiers::visualization::CreateVerticalFOVMarker(temp_view, i, range_for_viz));
    marker_array_rejects.markers.push_back(map_frontiers::visualization::CreateViewToFrontiersLine(temp_view, temp_frontier, i));


  }

  //History of selected views
  for(int i = 0; i < views_history.size(); i++){
    geometry_msgs::Pose temp_view;
    map_frontiers::conversions::ViewCandidateToGeometryPose(views_history[i], temp_view);
    geometry_msgs::Point temp_frontier;
    temp_frontier.x = views_history[i].o_x;
    temp_frontier.y = views_history[i].o_y;
    temp_frontier.z = views_history[i].o_z;
    views_set.poses.push_back(temp_view);

    // Create markers to represent the fov the view would have
    marker_array.markers.push_back(map_frontiers::visualization::CreateHorizontalFOVMarker(temp_view, i, range_for_viz));
    marker_array.markers.push_back(map_frontiers::visualization::CreateVerticalFOVMarker(temp_view, i, range_for_viz));
    marker_array.markers.push_back(map_frontiers::visualization::CreateViewToFrontiersLine(temp_view, temp_frontier, i));

    ROS_INFO_COND(verbose_, "View history %d %f %f %f %f", i, views_history[i].q_x, views_history[i].q_y, views_history[i].q_z, views_history[i].q_w);

  }
  
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

    // ROS_INFO_COND(verbose_, "View  %d %f %f %f %f", i, views[i].q_x, views[i].q_y, views[i].q_z, views[i].q_w);

  }
  views_set.header.frame_id = world_frame_;
  rejected_views_set.header.frame_id = world_frame_;
  pub_views.publish(views_set);
  pub_rejected_views.publish( rejected_views_set);
  pub_views_marker.publish(marker_array);

  pub_views_rejected_marker.publish( marker_array_rejects ) ;



  //publishing of rejected frontiers
  std::vector<Eigen::Vector3d> rejected_frontiers = m_view_generator.getRejectedFrontiers();
  for(int i=0; i< rejected_frontiers.size(); i++){
    pcl::PointXYZRGB point;
    point.x = rejected_frontiers[i].x();
    point.y = rejected_frontiers[i].y();
    point.z = rejected_frontiers[i].z();
    point.r = 0;
    point.g = 0;
    point.b = 0;
    rejected_frontiers_pointcloud.push_back(point);

  }


  rejected_frontiers_pointcloud.header.frame_id = world_frame_;
  pub_rejected.publish(rejected_frontiers_pointcloud);




}


//Called in poscallback after pose updated
void NBV_Selector::select_next_best_view(){
  
  ROS_INFO_COND(verbose_, "SELECTING VIEWS NOW");
  
  // ROS_INFO_COND(verbose_, "EMPTY SPACE");

  //evaluate views
  std::vector<float> values_views;
  int index_of_nbv = -1 ;
  float max_value_nbv = -1 ; 
  float image_value = -1 ; 
  float total_value = -1 ;
  float value_angular = 0 ;
  float dist_view = 0 ;
  float dist_min_views = -1 ;

  Eigen::Vector3d current_pos_vector( m_current_pos.x ,m_current_pos.y , m_current_pos.z );
  Eigen::Vector3d vel( m_vel_x, m_vel_y, m_vel_z);
  // ROS_INFO_COND(verbose_, "VEL VALUES ARE %f %f %f", m_vel_x, m_vel_y, m_vel_z);

  if (views.size() > 0){
    dist_min_views = m_view_evaluator.getMinViewDistance(views, current_pos_vector) ;
  }
  else{
    ROS_INFO_COND(verbose_, "NO VIEWS TO CHECK FOR DISTANCE VIEWS ");
  }
  ROS_INFO_COND(verbose_, "MIN DISTANCE FRONTIERS IS %f", dist_min_views);


  m_current_goal = m_current_pos;

  ROS_INFO_COND(verbose_, "EXAMINING %d", views.size());
  ROS_INFO_COND(verbose_, "ALPHA %f BETA %f GAMMA %d", m_alpha, m_beta, m_gamma);
  ros::Time total_view_evaluation_start = ros::Time::now();

  for(int i=0;i< views.size();i++){

    ROS_INFO_COND(verbose_, "GET VOXELS VIEW %d", i);


    ViewCandidate view = views[i] ; 
    std::vector<Eigen::Vector3d> visible_voxels; 
    Eigen::Vector3d view_vector = Eigen::Vector3d( view.x, view.y, view.z);
    Eigen::Quaterniond orient = Eigen::Quaterniond( view.q_x, view.q_y, view.q_z, view.q_w);

    ros::Time start_get_visible_voxels_lidar = ros::Time::now();

    // m_view_evaluator.getVisibleVoxels_LIDAR(
    // &visible_voxels, pos, orient) ;
    // ros::Time end_get_visible_voxels_lidar = ros::Time::now();
    // ros::Duration duration = end_get_visible_voxels_lidar - start_get_visible_voxels_lidar;
    // ROS_INFO_COND(timer_, "[NBV_Selector][getVisibleVoxels_LIDAR] %.4f s", duration.toSec());

    ROS_INFO_COND(verbose_, "COUNTING FRONTIERS VIEW %d", i);
    
    ros::Time start_view_evaluation = ros::Time::now();
    //temp_value = m_view_evaluator.count_frontiers_view(visible_voxels);
    //temp_value = m_view_evaluator.evaluate_view_image(visible_voxels);
    image_value = m_view_evaluator.inverseRayCast(view, frontiers_set ); 
    ROS_INFO_COND(verbose_, "IMAGE VALUE of  %d is %f", i, image_value);


    ros::Time end_view_evaluation = ros::Time::now();
    ros::Duration duration = end_view_evaluation - start_view_evaluation;
    ROS_INFO_COND(timer_, "[NBV_Selector][Evaluate View] %.4f s", duration.toSec());

    value_angular = m_view_evaluator.evaluate_view_angular( view, current_pos_vector, vel ) ; 
    
    //IF DISTANCE IS TAKEN  INTO ACCOUNT
    if ( m_gamma == true ){
      dist_view = (view_vector - current_pos_vector ).norm() ;
      value_angular = m_view_evaluator.evaluate_distance_angle_cost( value_angular, dist_min_views, dist_view ) ; 
    }

    ROS_INFO_COND(verbose_, "ANGLE COST VALUE of  %d is %f", i, value_angular);
    // temp_value_angular = ; 
    //ROS_INFO_COND(verbose_, "DIST/ANGLE COST VALUE of  %d is %f", i, temp_value_angular);

    total_value = m_alpha * image_value + m_beta * value_angular ; 
    ROS_INFO_COND(verbose_, "TOTAL VALUE OF  %d is %f", i, total_value);
    
    values_views.push_back(total_value);
    if( total_value > max_value_nbv){
      index_of_nbv = i;
      max_value_nbv = total_value;

    }

    // break ; 


  }


  //select views
  ros::Time total_view_evaluation_end = ros::Time::now();
  ros::Duration duration_total = total_view_evaluation_start - total_view_evaluation_end ; 
  ROS_INFO_COND(timer_, "[NBV_Selector][Evaluate View] Total %.4f s", duration_total.toSec());



  if (views.size() > 0){
    ROS_INFO_COND(verbose_, "UPDATING CURRENT GOAL ");
    ROS_INFO_COND(verbose_, "NEW GOAL IS VIEW %d", index_of_nbv);
    ROS_INFO_COND(verbose_, " %f %f %f ", views[index_of_nbv].x , views[index_of_nbv].y , views[index_of_nbv].z );
    ROS_INFO_COND(verbose_, "CURRENT POS IS %f %f %f", m_current_pos.x, m_current_pos.y, m_current_pos.z);

    views_history.push_back( views[index_of_nbv]) ;

    //Correct orientation of the goal
    ViewCandidate goal_corrected = views[index_of_nbv];
    float temp_number = goal_corrected.z ; 
    goal_corrected.z = goal_corrected.o_z ; 
    findOrientation(goal_corrected);
    goal_corrected.z = temp_number ; 

    //m_current_goal = views[index_of_nbv];
    m_current_goal = goal_corrected;
    
  }




}

void NBV_Selector::next_best_view_closest_frontier(){
  ROS_INFO_COND(verbose_, "CLOSEST FRONTIER MODE");
  
  // ROS_INFO_COND(verbose_, "EMPTY SPACE");

  //evaluate views
  std::vector<float> values_views;
  int index_of_nbv = -1 ;
  float max_value_nbv = -1 ; 
  float temp_value = -1 ; 
  m_current_goal = m_current_pos;
  float temp_value_angular = 0 ;
  float max_value_angular = 0 ; 
  ROS_INFO_COND(verbose_, "EXAMINING %d", views.size());
  for(int i=0;i< views.size();i++){

    ViewCandidate view = views[i] ; 
    std::vector<Eigen::Vector3d> visible_voxels; 
    Eigen::Vector3d pos = Eigen::Vector3d( view.x, view.y, view.z);

    Eigen::Vector3d current_pos_vector( m_current_pos.x ,m_current_pos.y , m_current_pos.z );
    Eigen::Vector3d vel( m_vel_x, m_vel_y, m_vel_z);
    temp_value_angular = m_view_evaluator.evaluate_view_angular( view, current_pos_vector, vel ) ; 


  }

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "nbv_selector_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");  

    system_parameters sys_params ; 
    /////////////// NAVIGATION
    sys_params.tolerance_distance = 0.2 ; //TO DO : CHECK UNITE

    sys_params.threshold_known = 0.0 ; //from Hardouin 0.3
    
    //////////////Planning parameters
    sys_params.alpha = 1.0 ;// image component weight
    sys_params.beta = 1.0 ;// angular/distance cost component weight 
    sys_params.gamma = 1 ;  // BOOLEAN : 0 or 1 /// IF 0 distance is NOT taken into account in the angular/distance component 
    sys_params.occlusion = false; 

    //////////////View Generation parameters
    sys_params.method_view_generation = "gradient" ; 
    sys_params.distance_min = 1;
    sys_params.distance_max = 2;
    sys_params.subsampling_views = 100 ;
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



