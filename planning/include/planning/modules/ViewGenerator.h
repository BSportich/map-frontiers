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
    float q_x ;
    float q_y;
    float q_z;
    float q_w;

    //origin of the generation
    int o_x;
    int o_y;
    int o_z;

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
    voxblox_map::VoxbloxMap m_map;

public:
    ViewGenerator(const std::string& method_name, float distance_min, float distance_max, const voxblox_map::VoxbloxMap& map);
    ViewGenerator(float distance_min, float distance_max);
    ViewGenerator();
    ~ViewGenerator();

    void generateViews(const std::vector<Eigen::Vector3d>& frontiers_set);
    void generateViews_sphere(const std::vector<Eigen::Vector3d>& frontiers_set);
    // void generateViews_echo(std::vector<Eigen::Vector3d> frontiers_set);
    // void generateViews_normals(std::vector<Eigen::Vector3d> frontiers_set);
    // void generateViews_gradients(std::vector<Eigen::Vector3d> frontiers_set);
    

    std::vector<ViewCandidate> getViewCandidates(){ return view_candidates; };
    std::string getMethod_type(){ return m_method_type; } ;



    

    //std::string method_type;
    
};

ViewGenerator::ViewGenerator(/* args */)
{
}

ViewGenerator::~ViewGenerator()
{
}


void findOrientation(ViewCandidate& vc);
