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
    int x;
    int y;
    int z;

    //orientation
    int q_x ;
    int q_y;
    int q_z;
    int q_w;

    //origin of the generation
    int o_x;
    int o_y;
    int o_z;

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


void findOrientation(ViewCandidate& vc, const Eigen::Vector3d& voxel_frontier);
