#include "planning/modules/ViewGenerator.h"
#include <math.h>


ViewGenerator::ViewGenerator(float distance_min, float distance_max){
    ViewGenerator("sphere", distance_min, distance_max);
}

ViewGenerator::ViewGenerator(const std::string& method_name, float distance_min, float distance_max){
    m_method_type = method_name;
    m_distance_max = distance_max;
    m_distance_min = m_distance_min;
    if(distance_min > distance_max) {

        //throw exception
        throw string(" Distances values are incorrect. Can not initialize ViewGenerator");
    }
}

ViewGenerator::generateViews( std::vector<Eigen::Vector3d> frontiers_set ){
    if(m_method_type == "sphere"){

        ViewGenerator::generateViews_sphere(std::vector<Eigen::Vector3d> frontiers_set );
    }
    else{
        throw string("Methods not defined ! ");
    }
}

ViewGenerator::generateViews_sphere_xyz( std::vector<Eigen::Vector3d> frontiers_set ){
    for(int i=0; i< frontiers_set.size(); i++){


        Eigen::Vector3d frontier = frontiers_set[i]
        temp_x = frontier.x();
        temp_y = frontier.y();
        temp_z = frontier.z(); 

        for(int j=0; j< max_sampling; j++){

            float formula = -1 ;

            while(formula < m_distance_min || formula > m_distance_max){
            
                float o_x = ( std::rand(0, m_distance_max) - m_distance_max) ;
                float o_y = ( std::rand(0, m_distance_max) - m_distance_max) ;
                float o_z = ( std::rand(0, m_distance_max) - m_distance_max) ;

                formula = (o_x - temp_x ) * (o_x - temp_x ) + (o_y - temp_y ) * (o_y - temp_y ) + (o_z - temp_z ) * (o_z - temp_z ) ;
            }

            view_candidates.push_back( ViewCandidate(o_x,o_y,o_z, 0,0,0,0, temp_x, temp_y, temp_z) ) ;

        }






    }
}

ViewGenerator::generateViews_sphere_sph_coord( std::vector<Eigen::Vector3d> frontiers_set ){
    for(int i=0; i< frontiers_set.size(); i++){


        Eigen::Vector3d frontier = frontiers_set[i]
        temp_x = frontier.x();
        temp_y = frontier.y();
        temp_z = frontier.z(); 

        for(int j=0; j< max_sampling; j++){

            float formula = -1 ;

            while(formula < m_distance_min || formula > m_distance_max){
            
                float r = ( std::rand(0, m_distance_max) - m_distance_max) ;
                float theta = ( std::rand(0, 1) * 2 * M_PI) ;
                float phi = ( std::rand(0, 1) * 2 * M_PI) ;

                //spherical coordinates
                float o_x = r * sin( theta ) * cos( phi ) ; 
                float o_y = r * sin( theta ) * sin ( phi );
                float o_z = r * cos( theta ) ;

                //cylindrical coordinates
                // float o_x = r * cos( theta ) ;
                // float o_y = r * sin( theta ) ;
                // float o_z = temp_z ?  ;


                formula = (o_x - temp_x ) * (o_x - temp_x ) + (o_y - temp_y ) * (o_y - temp_y ) + (o_z - temp_z ) * (o_z - temp_z ) ;
            }

            view_candidates.push_back( ViewCandidate(o_x,o_y,o_z, 0,0,0,0, temp_x, temp_y, temp_z) ) ;

        }






    }
}


