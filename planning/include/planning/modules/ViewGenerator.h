#pragma once
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <cstdlib>
#include <voxblox_map/voxblox_map.h>



struct ViewCandidate
{
    //position
    float x;
    float y;
    float z;

    //orientation
    double q_x ;
    double q_y;
    double q_z;
    double q_w;

    //origin of the generation
    float o_x;
    float o_y;
    float o_z;

    // ViewCandidate(int x, int y, int z, float qx, float qy, float qz, float qw, int ox, int oy, int oz)
    //     : x(x), y(y), z(z), q_x(qx), q_y(qy), q_z(qz), q_w(qw), o_x(ox), o_y(oy), o_z(oz) {}

};

class ViewGenerator
{
private:
    /* data */
    std::string m_method_type;
    std::vector<ViewCandidate> view_candidates;
    float m_distance_max; // max sampling distance from the frontiers
    float m_distance_min; // min sampling distance from the frontiers
    float m_angle_low ; // vertical angle below the drone
    float m_angle_high; // vertical angle above the drone 
    voxblox_map::VoxbloxMap m_map;

    int max_sampling;
    float robot_radius_ ; 

public:
    ViewGenerator(const std::string& method_name, float distance_min, float distance_max, const voxblox_map::VoxbloxMap& map, float robot_radius, float angle_low, float angle_high);
    ViewGenerator(){};
    ~ViewGenerator(){};

    void generateViews(const std::vector<Eigen::Vector3d>& frontiers_set);
    void generateViews_sphere(const std::vector<Eigen::Vector3d>& frontiers_set);
    // void generateViews_echo(std::vector<Eigen::Vector3d> frontiers_set);
    void generateViews_normals(const std::vector<Eigen::Vector3d>& frontiers_set);
    bool generateview_normal(const Eigen::Vector3d& frontier, float rotation, float& angle_diff, ViewCandidate& vc);

    void generateViews_gradients_ESDF(const std::vector<Eigen::Vector3d>& frontiers_set);


    bool verify_angle(const Eigen::Vector3d& frontier,const ViewCandidate& vc, float& angle);
    Eigen::Vector3d computeGradient(const Eigen::Vector3d& voxel);
    

    std::vector<ViewCandidate> getViewCandidates(){ return view_candidates; };
    std::string getMethod_type(){ return m_method_type; } ;
    bool isSafeView(const Eigen::Vector3d& voxel);



    

    //std::string method_type;
    
};



void findOrientation(ViewCandidate& vc);
