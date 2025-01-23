#include "planning/modules/ViewGenerator.h"
#include <math.h>


ViewGenerator::ViewGenerator(float distance_min, float distance_max)  
   : ViewGenerator("sphere", distance_min, distance_max, voxblox_map::VoxbloxMap()) {}

ViewGenerator::ViewGenerator(const std::string& method_name, float distance_min, float distance_max, const voxblox_map::VoxbloxMap& map){
    m_method_type = method_name;
    m_distance_max = distance_max;
    m_distance_min = distance_min;
    m_map = map;
    if(distance_min > distance_max) {

        //throw exception
        throw string(" Distances values are incorrect. Can not initialize ViewGenerator");
    }
}

ViewGenerator::generateViews(const std::vector<Eigen::Vector3d>& frontiers_set ){
    if(m_method_type == "sphere"){

        ViewGenerator::generateViews_sphere( frontiers_set );
    }
    else{
        throw std::invalid_argument("Methods not defined ! ");
    }
}

// ViewGenerator::generateViews_sphere_xyz( std::vector<Eigen::Vector3d> frontiers_set ){
//     for(int i=0; i< frontiers_set.size(); i++){


//         Eigen::Vector3d frontier = frontiers_set[i]
//         temp_x = frontier.x();
//         temp_y = frontier.y();
//         temp_z = frontier.z(); 

//         for(int j=0; j< max_sampling; j++){

//             float formula = -1 ;

//             while(formula < m_distance_min || formula > m_distance_max){
            
//                 float o_x = ( std::rand(0, m_distance_max) - m_distance_max) ;
//                 float o_y = ( std::rand(0, m_distance_max) - m_distance_max) ;
//                 float o_z = ( std::rand(0, m_distance_max) - m_distance_max) ;

//                 formula = (o_x - temp_x ) * (o_x - temp_x ) + (o_y - temp_y ) * (o_y - temp_y ) + (o_z - temp_z ) * (o_z - temp_z ) ;
//             }

//             view_candidates.push_back( ViewCandidate(o_x,o_y,o_z, 0,0,0,0, temp_x, temp_y, temp_z) ) ;

//         }






//     }
// }

ViewGenerator::generateViews_sphere(const std::vector<Eigen::Vector3d>& frontiers_set ){
    for(int i=0; i< frontiers_set.size(); i++){


        Eigen::Vector3d frontier = frontiers_set[i];
        float temp_x = frontier.x();
        float temp_y = frontier.y();
        float temp_z = frontier.z(); 

        for(int j=0; j< max_sampling; j++){

            float formula = -1 ;
            bool isFree = false;

            while(isFree){
            
                float r = ( std::rand(0, m_distance_max - m_distance_min ) + m_distance_min) ;
                float theta = ( std::rand(0, 1) * 2 * M_PI) ;
                float phi = ( std::rand(0, 1) * 2 * M_PI) ;

                //spherical coordinates
                float o_x = temp_x + r * sin( theta ) * cos( phi ) ; 
                float o_y = temp_y + r * sin( theta ) * sin ( phi );
                float o_z = temp_z + r * cos( theta ) ;

                //cylindrical coordinates
                // float o_x = r * cos( theta ) ;
                // float o_y = r * sin( theta ) ;
                // float o_z = temp_z ? ;


                current_state = m_map.getVoxelState_TSDF(voxel);
                isFree = (current_state == voxblox_map::VoxbloxMap::FREE);


                //formula = (o_x - temp_x ) * (o_x - temp_x ) + (o_y - temp_y ) * (o_y - temp_y ) + (o_z - temp_z ) * (o_z - temp_z ) ;
            }

            ViewCandidate vc = {o_x,o_y,o_z, 0,0,0,0, temp_x, temp_y, temp_z};
            
            findOrientation(vc);

            view_candidates.push_back( vc ) ;

        }






    }
}


void findOrientation(ViewCandidate& vc){

    Eigen::Vector3d view_pos( vc.x, vc.y, vc.z ) ;
    Eigen::Vector3d frontier_pos( vc.o_x, vc.o_y, vc.o_z ) ;

    //Computes direction vector
    Eigen::Vector3d direction = frontier_pos - view_pos ; 
    
    // Normalize the direction vector to get the forward vector
    Eigen::Vector3d forward = direction.normalized();

    float heading_angle =  atan2( forward.y, forward.x ) ;
    float yaw = heading_angle ; 
    // Compute the right vector as the cross product of up and forward

    float pitch = asin( forward.z ) ;
    float roll = 0 ; 

    qx = sin(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0) - cos(roll/2.0) * sin(pitch/2.0) * sin(yaw/2.0) ;
    qy = cos(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0) + sin(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0) ;
    qz = cos(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0) - sin(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0) ;
    qw = cos(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0) ;

    vc.qx = qx;
    vc.qy = qy;
    vc.qz = qz;
    vc.qw = qw;


}