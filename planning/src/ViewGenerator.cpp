#include "planning/modules/ViewGenerator.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include "ros/ros.h"


ViewGenerator::ViewGenerator(const std::string& method_name, float distance_min, float distance_max, const voxblox_map::VoxbloxMap& map, float robot_radius){
    m_method_type = method_name;
    m_distance_max = distance_max;
    m_distance_min = distance_min;
    m_map = map;
    robot_radius_ = robot_radius ; 

    max_sampling = 1;
    if(distance_min > distance_max) {

        //throw exception
        throw std::string(" Distances values are incorrect. Can not initialize ViewGenerator");
    }
}

void ViewGenerator::generateViews(const std::vector<Eigen::Vector3d>& frontiers_set ){
    if(m_method_type == "sphere"){

        ViewGenerator::generateViews_sphere( frontiers_set );
    }
    else{
        throw std::invalid_argument("Methods not defined ! ");
    }
}


void ViewGenerator::generateViews_sphere(const std::vector<Eigen::Vector3d>& frontiers_set ){

    view_candidates.clear();

    for(int i=0; i< frontiers_set.size(); i++){


        Eigen::Vector3d frontier = frontiers_set[i];
        float temp_x = frontier.x();
        float temp_y = frontier.y();
        float temp_z = frontier.z(); 

        for(int j=0; j< max_sampling; j++){

            float formula = -1 ;
            bool isFree = false;
            float o_x = 0;
            float o_y = 0 ;
            float o_z = 0 ; 

            while((isFree == false)){
            
                float r = ( (static_cast<float>(rand()) / RAND_MAX) * ( m_distance_max - m_distance_min ) + m_distance_min) ;
                float theta = ( (static_cast<float>(rand()) / RAND_MAX)  * 2 * M_PI) ;
                float phi = ( (static_cast<float>(rand()) / RAND_MAX) * 2 * M_PI) ;

                //spherical coordinates
                o_x = temp_x + r * sin( theta ) * cos( phi ) ; 
                o_y = temp_y + r * sin( theta ) * sin ( phi );
                o_z = temp_z + r * cos( theta ) ;

                //cylindrical coordinates
                // float o_x = r * cos( theta ) ;
                // float o_y = r * sin( theta ) ;
                // float o_z = temp_z ? ;

                Eigen::Vector3d voxel  = Eigen::Vector3d( o_x, o_y, o_z);
                unsigned char current_state = m_map.getVoxelState_ESDF(voxel);
                isFree = (current_state == voxblox_map::VoxbloxMap::FREE);
                //isFree = isSafeView(voxel); // never converges
                ROS_INFO(" Is safe ?%s ", isFree ? "true " : "false");
                //ROS_INFO("Generating views %f %f %f ", o_x, o_y, o_z );


                //formula = (o_x - temp_x ) * (o_x - temp_x ) + (o_y - temp_y ) * (o_y - temp_y ) + (o_z - temp_z ) * (o_z - temp_z ) ;
            }

            ViewCandidate vc = {o_x,o_y,o_z, 0,0,0,0, temp_x, temp_y, temp_z};
            
            findOrientation(vc);

            view_candidates.push_back( vc ) ;

        }






    }
}

void ViewGenerator::generateViews_normals(const std::vector<Eigen::Vector3d>& frontiers_set ){
    view_candidates.clear();

    for(int i=0; i< frontiers_set.size(); i++){


        Eigen::Vector3d frontier = frontiers_set[i];
        float temp_x = frontier.x();
        float temp_y = frontier.y();
        float temp_z = frontier.z(); 


        m_map


        

    }



}


// void findOrientation(ViewCandidate& vc){

//     Eigen::Vector3d view_pos( vc.x, vc.y, vc.z ) ;
//     Eigen::Vector3d frontier_pos( vc.o_x, vc.o_y, vc.o_z ) ;

//     //Computes direction vector
//     Eigen::Vector3d direction =  frontier_pos - view_pos ; 
    
//     // Normalize the direction vector to get the forward vector
//     Eigen::Vector3d forward = direction.normalized();

//     float heading_angle =  atan2( forward.y(), forward.x() ) ;
//     float yaw = heading_angle ; 
//     // Compute the right vector as the cross product of up and forward

//     float pitch = asin( forward.z() ) ;
//     float roll = 0 ; 

//     float qx = sin(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0) - cos(roll/2.0) * sin(pitch/2.0) * sin(yaw/2.0) ;
//     float qy = cos(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0) + sin(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0) ;
//     float qz = cos(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0) - sin(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0) ;
//     float qw = cos(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0) ;

//     vc.q_x = qx;
//     vc.q_y = qy;
//     vc.q_z = qz;
//     vc.q_w = qw;


// }

void findOrientation(ViewCandidate& vc){
    // Convert the points to tf2::Vector3 for easier vector operations
    tf2::Vector3 view_pos(vc.x, vc.y, vc.z);
    tf2::Vector3 frontier_pos(vc.o_x, vc.o_y, vc.o_z);

    // Compute the direction vector from point1 to point2
    tf2::Vector3 direction = frontier_pos - view_pos;
    
    // Normalize the direction vector (to ensure it's a unit vector)
    direction.normalize();

    // The forward direction that "view_pos" should align with (now along the X-axis)
    tf2::Vector3 forward(1.0, 0.0, 0.0); // Align with the X-axis

    // Compute the axis of rotation (cross product of forward and direction)
    tf2::Vector3 axis = forward.cross(direction);
    axis.normalize();  // Normalize the axis

    // Compute the angle of rotation (dot product gives cosine of the angle)
    float dot = forward.dot(direction);
    // Clamp the dot product to avoid precision errors in the acos function
    dot = std::min(1.0f, std::max(-1.0f, dot));

    // Calculate the angle between the vectors
    float angle = acos(dot);

    // Compute the quaternion representing the rotation
    tf2::Quaternion rotation;
    rotation.setRotation(axis, angle);

    vc.q_x = rotation.x();
    vc.q_y = rotation.y();
    vc.q_z = rotation.z();
    vc.q_w = rotation.w();
}

bool ViewGenerator::isSafeView(const Eigen::Vector3d& voxel){
    //isFree = (current_state == voxblox_map::VoxbloxMap::FREE);
    //const voxblox::EsdfVoxel& voxel = 
    float dist = m_map.getVoxelDistance_ESDF(voxel) ;
    if( dist < (robot_radius_ * m_map.getVoxelSize() * 0.5 ) ){
        return false;
    }
    for(int i= -std::ceil(robot_radius_/2) ; i < std::ceil(robot_radius_/2) ; i++){
        for(int j= -std::ceil(robot_radius_/2) ; i < std::ceil(robot_radius_/2) ; i++){
            for(int k= -std::ceil(robot_radius_/2) ; i < std::ceil(robot_radius_/2) ; k++){

                Eigen::Vector3d shift = Eigen::Vector3d(i,j,k);
                dist = m_map.getVoxelDistance_ESDF(voxel + shift) ;
                if( dist < m_map.getVoxelSize() ){
                    return false;
                }

            }
        }
    }
    return true;

}


// ViewGenerator::ViewGenerator(/* args */)
// {
// }

// ViewGenerator::~ViewGenerator()
// {
// }
