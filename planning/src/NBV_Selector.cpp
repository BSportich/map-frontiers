#include <Eigen/Eigen>
#include <iostream>
#include "planning/modules/ViewGenerator.h"
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <voxblox_map/voxblox_map.h>
#include <voxblox_msgs/Layer.h>

struct Frontier
{
    //position
    int x;
    int y;
    int z;

    //orientation
    int q_w ;
    int q_x;
    int q_y;
    int q_z;

};



class NBV_Selector
{
private:
    /* data */
    voxblox_map::VoxbloxMap m_map;


    ViewGenerator m_view_generator;
    std::vector<Frontier> frontiers;
    bool m_frontier6;
    bool m_surface_frontiers;
    Frontier m_current_goal;
    Frontier m_current_pos;
    bool m_availability;

    //ros communication
    ros::NodeHandle n;
    ros::Subscriber sub_map_tsdf;
    ros::Subscriber sub_map_esdf;
    ros::Subscriber sub_pos;
    ros::Publisher pub_goal;

    //team analysis
    int m_team_size;
    int m_team_id;
    std::vector<int> m_team;
    std::unordered_map<int, Frontier> m_team_pos;


    //neighbours
    Eigen::Vector3d c_neighbor_voxels_[26]; 

    //states
    const static unsigned char AVAILABLE = 0;  // NOLINT
    const static unsigned char BUSY = 1;      // NOLINT



public:
    NBV_Selector();
    NBV_Selector(const ViewGenerator& vg, FloatingPoint voxel_size, size_t voxels_per_side, int team_id, std::vector<int> robot_team);
    void updateFrontiers();
    bool isFrontierVoxel_ESDF(const Eigen::Vector3d& voxel);


    //Tests functions
    //void test_publish();

    //callbacks
    void tSDFCallback(const voxblox_msgs::Layer& layer_msg);
    void eSDFCallback(const voxblox_msgs::Layer& layer_msg);
    void posCallback(const std_msgs::String::ConstPtr& msg);
  

    ~NBV_Selector();
};

NBV_Selector::NBV_Selector(const ViewGenerator& vg, FloatingPoint voxel_size, size_t voxels_per_side, int team_id, std::vector<int> robot_team): 
{
    
    //initialization
    int vs = 1;
    m_team_id = team_id ;
    m_team_size = robot_team.size() ;
    m_team( robot_team );
    m_team_pos();
    m_availability = AVAILABLE;

    //map
    m_map = voxbloxmap::VoxbloxMap(voxel_size, voxels_per_side);

    //modules
    m_view_generator(vg);

    //frontiers
    frontiers();

    //ros initialization
    ros::init(argc, argv, "NBV_selector_node robot ");
    sub_map_tsdf = n.subscribe("tsdf_map_out", 1000, tSDFCallback);
    sub_map_esdf = n.subscribe("esdf_map_out", 1000, eSDFCallback);
    sub_pos = n.subscribe("pos", 1000, posCallback);
    pub_goal = n.advertise<std_msgs::String>("pos_goal", 1000); //to redefine msg type

    if(frontier6 == true){
        c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
        c_neighbor_voxels_[1] = Eigen::Vector3d(-vs, 0, 0);
        c_neighbor_voxels_[2] = Eigen::Vector3d(0, vs, 0);
        c_neighbor_voxels_[3] = Eigen::Vector3d(0, -vs, 0);
        c_neighbor_voxels_[4] = Eigen::Vector3d(0, 0, vs);
        c_neighbor_voxels_[5] = Eigen::Vector3d(0, 0, -vs);
    }
    else{
        c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
        c_neighbor_voxels_[1] = Eigen::Vector3d(vs, vs, 0);
        c_neighbor_voxels_[2] = Eigen::Vector3d(vs, -vs, 0);
        c_neighbor_voxels_[3] = Eigen::Vector3d(vs, 0, vs);
        c_neighbor_voxels_[4] = Eigen::Vector3d(vs, vs, vs);
        c_neighbor_voxels_[5] = Eigen::Vector3d(vs, -vs, vs);
        c_neighbor_voxels_[6] = Eigen::Vector3d(vs, 0, -vs);
        c_neighbor_voxels_[7] = Eigen::Vector3d(vs, vs, -vs);
        c_neighbor_voxels_[8] = Eigen::Vector3d(vs, -vs, -vs);
        c_neighbor_voxels_[9] = Eigen::Vector3d(0, vs, 0);
        c_neighbor_voxels_[10] = Eigen::Vector3d(0, -vs, 0);
        c_neighbor_voxels_[11] = Eigen::Vector3d(0, 0, vs);
        c_neighbor_voxels_[12] = Eigen::Vector3d(0, vs, vs);
        c_neighbor_voxels_[13] = Eigen::Vector3d(0, -vs, vs);
        c_neighbor_voxels_[14] = Eigen::Vector3d(0, 0, -vs);
        c_neighbor_voxels_[15] = Eigen::Vector3d(0, vs, -vs);
        c_neighbor_voxels_[16] = Eigen::Vector3d(0, -vs, -vs);
        c_neighbor_voxels_[17] = Eigen::Vector3d(-vs, 0, 0);
        c_neighbor_voxels_[18] = Eigen::Vector3d(-vs, vs, 0);
        c_neighbor_voxels_[19] = Eigen::Vector3d(-vs, -vs, 0);
        c_neighbor_voxels_[20] = Eigen::Vector3d(-vs, 0, vs);
        c_neighbor_voxels_[21] = Eigen::Vector3d(-vs, vs, vs);
        c_neighbor_voxels_[22] = Eigen::Vector3d(-vs, -vs, vs);
        c_neighbor_voxels_[23] = Eigen::Vector3d(-vs, 0, -vs);
        c_neighbor_voxels_[24] = Eigen::Vector3d(-vs, vs, -vs);
        c_neighbor_voxels_[25] = Eigen::Vector3d(-vs, -vs, -vs);
    }

    //ros::spin()
}

bool NBV_Selector::isFrontierVoxel_ESDF(const Eigen::Vector3d& voxel){
  unsigned char voxel_state;
  if ( m_frontier6 ) {
    for (int i = 0; i < 6; ++i) {
      voxel_state = m_map->getVoxelState_ESDF(voxel + c_neighbor_voxels_[i]);
      if (voxel_state == voxblox_map::VoxbloxMap::UNKNOWN) {
        continue;
      }
      if (m_surface_frontiers ) {
        return voxel_state == voxblox_map::VoxbloxMap::OCCUPIED;
      } else {
        return true;
      }
    }
  } else {
    for (int i = 0; i < 26; ++i) {
      voxel_state = m_map->getVoxelState_ESDF(voxel + c_neighbor_voxels_[i]);
      if (voxel_state == voxblox_map::VoxbloxMap::UNKNOWN) {
        continue;
      }
      if ( m_surface_frontiers ) {
        return voxel_state == voxblox_map::VoxbloxMap::OCCUPIED;
      } else {
        return true;
      }
    }
  }
  return false;

}


void NBV_Selector::updateFrontiers(){

    ROS_INFO("Updated frontiers: %lu found", frontiers.size());

}

NBV_Selector::~NBV_Selector()
{
}

void NBV_Selector::tSDFCallback(const voxblox_msgs::Layer& layer_msg){
  timing::Timer receive_map_timer("map/receive_tsdf");

  bool success =
      voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(layer_msg, m_map.get_tsdf_map_pointer()->getTsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid TSDF map message!");
  } else {
    ROS_INFO_ONCE("Got an TSDF map from ROS topic!");
    }
  


}

void NBV_Selector::eSDFCallback(const voxblox_msgs::Layer& layer_msg){
  timing::Timer receive_map_timer("map/receive_esdf");

  bool success =
      voxblox::deserializeMsgToLayer<voxblox::EsdfVoxel>(layer_msg, m_map.get_esdf_map_pointer()->getEsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid ESDF map message!");
  } else {
    ROS_INFO_ONCE("Got an ESDF map from ROS topic!");
    }
  


}




void NBV_Selector::posCallback(const std_msgs::String::ConstPtr& msg);


int main(int argc, char** argv) {
    //ros::init(argc, argv, "nbv_selector_node");
    ViewGenerator vg = ViewGenerator();
    FloatingPoint voxel_size = 0.2; //taken for default value in the code of tsdf_map.h and esdf_map.h
    size_t voxels_per_side = 16u ; // taken from default value in the code of tsdf_map.h and esdf_map.h
    int team_id = 1;
    std::vector<int> robot_team ; 
    robot_team.push_back(team_id);
    NBV_Selector nbv_selector = NBV_Selector(vg, voxel_size, voxels_per_side, team_id,  robot_team);
    ros::spin();
    return 0;
}
