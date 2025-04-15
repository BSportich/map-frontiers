// utils.h
#pragma once  // or use include guards

#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <vector>
#include <sstream>
// Function declarations

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

struct BoundingBox
{
    //MAP PARAMETERS
    float x_max;
    float x_min;

    float y_max;
    float y_min;

    float z_max;
    float z_min;
    
};


bool verify_angle(const Eigen::Vector3d& frontier, const ViewCandidate& vc, float& angle, float angle_low, float angle_high);
bool isCorrectPos(const Eigen::Vector3d& pos,  const BoundingBox& bb);
void printVectorOneLine(const std::vector<double>& vec);


bool verify_angle(const Eigen::Vector3d& frontier, const ViewCandidate& vc, float& angle, float angle_low, float angle_high){

    float reduction_angle = 15 ;
    //if find orientation failed to find a direction
    if( vc.q_x == 0 &&  vc.q_y == 0 && vc.q_z == 0 && vc.q_w == 0 ){
        return false;
    }

    double vertical_angle_rad = 0 ;
    double vertical_angle_deg = 0 ;
    Eigen::Vector3d next_voxel( vc.x, vc.y, vc.z );

    //Compute direction with frontier
    Eigen::Vector3d direction_original = (frontier - next_voxel).normalized() ; // from view candidate, towards frontier
    Eigen::Vector3d direction_proj( direction_original.x(), direction_original.y(), 0) ;
    direction_proj.normalize() ;
    
    //Compute signed vertical angle
    //vertical_angle_rad = std::atan2(direction_original.z(), direction_original.head<2>().norm());
    vertical_angle_rad = std::atan2(direction_original.z(), direction_proj.norm()); //same as last line
    vertical_angle_deg = vertical_angle_rad * (180.0 / M_PI);

    // ROS_INFO("Angle found is function is  %f", vertical_angle_deg);
    angle = vertical_angle_deg;
    if( (vertical_angle_deg < angle_high - reduction_angle ) && (vertical_angle_deg > angle_low + reduction_angle) ){
        angle = vertical_angle_deg;
        return true;
    }

    return false;

}

bool isCorrectPos(const Eigen::Vector3d& pos, const BoundingBox& bb){

    if ( pos.z() >= bb.z_max || pos.z() <= bb.z_min ){
        return false;
    }

    if ( pos.y() >= bb.y_max || pos.y() <= bb.y_min ){
        return false;
    }

    if ( pos.x() >= bb.x_max || pos.x() <= bb.x_min ){
        return false;
    }

    return true ; 

  
}



void printVectorOneLine(const std::vector<int>& vec) {
    std::stringstream ss;
    ss << "[ ";
    for (int val : vec) {
        ss << val << " ";
    }
    ss << "]";
    ROS_INFO_STREAM("Vector: " << ss.str());
}
