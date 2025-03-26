#include "planning/modules/ViewGenerator.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include "ros/ros.h"
#include <algorithm>


ViewGenerator::ViewGenerator(const std::string& method_name, float distance_min, float distance_max, const voxblox_map::VoxbloxMap& map, float robot_radius, float angle_low, float angle_high){
    m_method_type = method_name;
    m_distance_max = distance_max;
    m_distance_min = distance_min;
    m_map = map;
    robot_radius_ = robot_radius ; 
    m_angle_low = angle_low ;
    m_angle_high = angle_high ; 

    rejected_frontiers = std::vector<Eigen::Vector3d>();
    max_sampling = 1;
    if(distance_min > distance_max) {

        //throw exception
        throw std::string(" Distances values are incorrect. Can not initialize ViewGenerator");
    }

    float vs = m_map.getVoxelSize() ; 
    c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
    c_neighbor_voxels_[1] = Eigen::Vector3d(-vs, 0, 0);
    c_neighbor_voxels_[2] = Eigen::Vector3d(0, vs, 0);
    c_neighbor_voxels_[3] = Eigen::Vector3d(0, -vs, 0);
    c_neighbor_voxels_[4] = Eigen::Vector3d(0, 0, vs);
    c_neighbor_voxels_[5] = Eigen::Vector3d(0, 0, -vs);
    
    c_neighbor_voxels_[6] = Eigen::Vector3d(vs, 0, -vs);
    c_neighbor_voxels_[7] = Eigen::Vector3d(vs, vs, -vs);
    c_neighbor_voxels_[8] = Eigen::Vector3d(vs, -vs, -vs);
    c_neighbor_voxels_[9] = Eigen::Vector3d(vs, vs, 0);
    c_neighbor_voxels_[10] = Eigen::Vector3d(vs, -vs, 0);
    c_neighbor_voxels_[11] = Eigen::Vector3d(vs, 0, vs);
    c_neighbor_voxels_[12] = Eigen::Vector3d(0, vs, vs);
    c_neighbor_voxels_[13] = Eigen::Vector3d(0, -vs, vs);
    c_neighbor_voxels_[14] = Eigen::Vector3d(vs, vs, vs);
    c_neighbor_voxels_[15] = Eigen::Vector3d(0, vs, -vs);
    c_neighbor_voxels_[16] = Eigen::Vector3d(0, -vs, -vs);
    c_neighbor_voxels_[17] = Eigen::Vector3d(vs, -vs, vs);
    c_neighbor_voxels_[18] = Eigen::Vector3d(-vs, vs, 0);
    c_neighbor_voxels_[19] = Eigen::Vector3d(-vs, -vs, 0);
    c_neighbor_voxels_[20] = Eigen::Vector3d(-vs, 0, vs);
    c_neighbor_voxels_[21] = Eigen::Vector3d(-vs, vs, vs);
    c_neighbor_voxels_[22] = Eigen::Vector3d(-vs, -vs, vs);
    c_neighbor_voxels_[23] = Eigen::Vector3d(-vs, 0, -vs);
    c_neighbor_voxels_[24] = Eigen::Vector3d(-vs, vs, -vs);
    c_neighbor_voxels_[25] = Eigen::Vector3d(-vs, -vs, -vs);
}

void ViewGenerator::generateViews(const std::vector<Eigen::Vector3d>& frontiers_set ){
    if(m_method_type == "sphere"){

        ViewGenerator::generateViews_sphere( frontiers_set );
    }
    else if(m_method_type == "gradient"){
        //ViewGenerator::generateViews_gradients_ESDF( frontiers_set ) ; 
        ViewGenerator::generateViews_normals(frontiers_set);
    }
    else{
        throw std::invalid_argument("Methods not defined ! ");
    }
}


void ViewGenerator::generateViews_sphere(const std::vector<Eigen::Vector3d>& frontiers_set ){

    view_candidates.clear();

    for(int i=0; i< frontiers_set.size(); i++){

        int numberOfOccupiedVoxels = 0;
        int numberOfUnknownVoxels = 0;

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
                ROS_INFO_THROTTLE(10, "Looking for an unoccupied view...");
            
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
                if (current_state == voxblox_map::VoxbloxMap::OCCUPIED)
                    numberOfOccupiedVoxels++;
                if (current_state == voxblox_map::VoxbloxMap::UNKNOWN)
                    numberOfUnknownVoxels++;
                //ROS_INFO("Generating views %f %f %f ", o_x, o_y, o_z );


                //formula = (o_x - temp_x ) * (o_x - temp_x ) + (o_y - temp_y ) * (o_y - temp_y ) + (o_z - temp_z ) * (o_z - temp_z ) ;
            }

            ROS_INFO("While looking for an unoccupied view, found %i occupied and %i unknown voxels", numberOfOccupiedVoxels, numberOfUnknownVoxels);

            ViewCandidate vc = {o_x,o_y,o_z, 0,0,0,0, temp_x, temp_y, temp_z};
            
            findOrientation(vc);

            view_candidates.push_back( vc ) ;

        }






    }
}

void ViewGenerator::generateViews_gradients_ESDF(const std::vector<Eigen::Vector3d>& frontiers_set){
    view_candidates.clear();
    float step_size = 0.5 ; 
    //int nb_steps = m_distance_max / m_map.getVoxelSize() ;
    int nb_steps = m_distance_max / step_size ;
    Eigen::Vector3d gradient ; 
    Eigen::Vector3d last_voxel, next_voxel ; 
    float distance = 0 ;
    bool isSafeView_bool ; 
    double vertical_angle_rad = 0 ;
    double vertical_angle_deg = 0 ;


    for(int i=0; i< frontiers_set.size(); i++){


        Eigen::Vector3d frontier = frontiers_set[i];
        next_voxel = frontier ; 

        ROS_INFO(" Generating view for frontier %d ",i);
        for(int j=0;j<= nb_steps; j++ ){
            
            last_voxel = next_voxel ;
            //find gradient
            gradient = computeGradient(last_voxel) ; 

            //find corresponding voxel
            //m_map.getVoxelCenter_ESDF( &next_voxel, (gradient + last_voxel)) ;
            next_voxel = (last_voxel + gradient) ; 
            distance = (frontier - next_voxel).norm() * m_map.getVoxelSize(); 
            ROS_INFO(" Moved from %f to %f now at distance %f", m_map.getDistancePrecise_ESDF(last_voxel), m_map.getDistancePrecise_ESDF(next_voxel), distance );
            
            //Angle verification
            //Compute direction with frontier
            // Eigen::Vector3d direction_original = (frontier - next_voxel).normalized() ; 
            // Eigen::Vector3d direction_proj( direction_original.x(), direction_original.y(), 0) ;
            // direction_proj.normalize() ;

            // //Compute signed vertical angle
            // vertical_angle_rad = std::atan2(direction_original.z(), direction_original.head<2>().norm());
            // //double angle_rad_atan2 = std::atan2(direction_original.z(), direction_proj.norm()); //same as last line
            // vertical_angle_deg = vertical_angle_rad * (180.0 / M_PI);
            



            
            
            //isSafeView_bool = isSafeView(next_voxel);
            isSafeView_bool = true ; 
            if( (distance  >  (( m_distance_max + m_distance_min)/2)) && (isSafeView_bool) ){
                break; 
            } 


        }

        if(isSafeView_bool){

            ViewCandidate vc = { next_voxel.x() , next_voxel.y() , next_voxel.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
            findOrientation( vc );
            ROS_INFO(" Angle found is %f", vertical_angle_deg);
            
            view_candidates.push_back( vc );


        }
        else{
            ROS_INFO("Impossible to generate view for frontier %d ",i);

            
        }



        

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

// void findOrientation(ViewCandidate& vc){
//     // Convert the points to tf2::Vector3 for easier vector operations
//     tf2::Vector3 view_pos(vc.x, vc.y, vc.z);
//     tf2::Vector3 frontier_pos(vc.o_x, vc.o_y, vc.o_z);

//     // Compute the direction vector from point1 to point2
//     tf2::Vector3 direction = frontier_pos - view_pos;

//     if (direction.length2() <= 0.0) {
//         ROS_INFO("Direction vector is zero! Cannot normalize.");
//         return;
//     }
    
//     // Normalize the direction vector (to ensure it's a unit vector)
//     direction.normalize();

//     // The forward direction that "view_pos" should align with (now along the X-axis)
//     tf2::Vector3 forward(1.0, 0.0, 0.0); // Align with the X-axis

//     // Compute the axis of rotation (cross product of forward and direction)
//     tf2::Vector3 axis = forward.cross(direction);

//     if (axis.length2() <= 0.0) {
//         ROS_INFO("Rotation axis is zero! Possible parallel vectors.");
//         return;
//     }

//     axis.normalize();  // Normalize the axis

//     // Compute the angle of rotation (dot product gives cosine of the angle)
//     float dot = forward.dot(direction);
//     // Clamp the dot product to avoid precision errors in the acos function
//     dot = std::min(1.0f, std::max(-1.0f, dot));

//     // Calculate the angle between the vectors
//     float angle = acos(dot);

//     // Compute the quaternion representing the rotation
//     tf2::Quaternion rotation;
//     rotation.setRotation(axis, angle);

//     if( (std::isnan(rotation.x())) ||  (std::isnan(rotation.y())) || (std::isnan(rotation.z())) || (std::isnan(rotation.w()))  ){
        
//     }

//     vc.q_x = rotation.x();
//     vc.q_y = rotation.y();
//     vc.q_z = rotation.z();
//     vc.q_w = rotation.w();
// }

void findOrientation(ViewCandidate& vc) { // CHATGPT
    // Convert the points to tf2::Vector3 for easier vector operations
    tf2::Vector3 view_pos(vc.x, vc.y, vc.z);
    tf2::Vector3 frontier_pos(vc.o_x, vc.o_y, vc.o_z);

    // Compute the direction vector from view_pos to frontier_pos
    tf2::Vector3 direction = frontier_pos - view_pos;

    if (direction.length2() <= 0.0) {
        ROS_WARN("Direction vector is zero! Cannot normalize.");
        return;
    }
    
    direction.normalize(); // Ensure it's a unit vector

    // Define the forward vector (assuming object initially faces +X)
    tf2::Vector3 forward(1.0, 0.0, 0.0); // Default reference

    // Compute rotation axis using cross product
    tf2::Vector3 axis = forward.cross(direction);

    // Compute the dot product for the angle
    float dot = forward.dot(direction);
    dot = std::max(-1.0f, std::min(1.0f, dot)); // Prevent floating-point precision issues

    // Handle parallel vectors (cross product = 0)
    tf2::Quaternion rotation;
    if (axis.length2() <= 0.0) {
        ROS_WARN("Rotation axis is zero! Vectors are parallel.");

        // If vectors are identical (dot ≈ 1), set identity rotation
        if (dot > 0.9999f) {
            rotation = tf2::Quaternion(0, 0, 0, 1);
        }
        // If vectors are opposite (dot ≈ -1), apply a 180-degree rotation around Z or Y
        else {
            tf2::Vector3 perpendicular(0, 0, 1); // Stable perpendicular axis
            rotation.setRotation(perpendicular, M_PI);
        }
    } else {
        axis.normalize();
        float angle = acos(dot);
        rotation.setRotation(axis, angle);
    }

    // Prevent NaN values
    if (std::isnan(rotation.x()) || std::isnan(rotation.y()) ||
        std::isnan(rotation.z()) || std::isnan(rotation.w())) {
        ROS_WARN("Quaternion contains NaN! Resetting to identity.");
        ROS_ERROR("Quaternion contains NaN! Resetting to identity.");
        rotation = tf2::Quaternion(0, 0, 0, 1);
    }

    // Store the computed quaternion in ViewCandidate
    vc.q_x = rotation.x();
    vc.q_y = rotation.y();
    vc.q_z = rotation.z();
    vc.q_w = rotation.w();
}


void ViewGenerator::generateViews_normals(const std::vector<Eigen::Vector3d>& frontiers_set){

    double vertical_angle_rad = 0 ;
    double vertical_angle_deg = 0 ;
    bool generated = false;
    float angle_diff; 
    float nb_rotation = 0 ; 
    float nb_sucess = 0 ; 
    float total = 0 ;

    view_candidates.clear();
    rejected_frontiers.clear();
    rejected_view_candidates.clear();

    for(int i=0; i< frontiers_set.size(); i++){

        ViewCandidate vc; 
        Eigen::Vector3d frontier = frontiers_set[i];
        ROS_INFO(" Generating view for frontier %d ",i);
        generated = generateview_normal(frontier, 0, angle_diff, vc);

        //if worked 
        if(generated){
            view_candidates.push_back( vc );
            // ROS_INFO(" %d Found view  %f %f %f ",i, vc.x, vc.y, vc.z);
            nb_sucess = nb_sucess +1 ; 
            continue;
        }

        if(generated == false && angle_diff == 0 ){
            ROS_INFO(" FAILURE : Gradient failed  %d ",i);
            rejected_frontiers.push_back( frontier ) ;
            continue ; 

        }
        

        // ROS_INFO(" ROTATION %d ",i);
        // // search for new view with a corrected vertical angle
        // generated = generateview_normal(frontier, (2* angle_diff) , angle_diff, vc);
        // //if worked
        // if(generated){
        //     view_candidates.push_back( vc );
        //     nb_rotation = nb_rotation +1 ;
        //     continue;
        // }

        //search for new view with a different horizontal angle

        //if didn't manage to find a solution
        rejected_frontiers.push_back( frontier ) ; 


        
    }
    total = (nb_rotation + nb_sucess) / frontiers_set.size() ; 
    ROS_INFO(" SUCCESS RATE VIEWS IS %f ",total);



        
     

}

bool ViewGenerator::generateview_normal(const Eigen::Vector3d& frontier, float rotation, float& angle_diff, ViewCandidate& vc){
    ViewCandidate min_view ; 
    ViewCandidate mid_view;
    ViewCandidate max_view ;
    ViewCandidate rejected_view;
    
    Eigen::Vector3d close_point;
    Eigen::Vector3d mid_point;
    Eigen::Vector3d far_point;

    float angle_min = 0;
    float angle_mid = 0;
    float angle_max = 0;

    Eigen::Vector3d gradient ; 
    Eigen::Vector3d current_pos ;
    Eigen::Vector3d vector_director ;  
    float distance = 0 ;
    float angle = -180 ; 
    bool isSafeView_bool = false; 
    bool isAngleok = true;

    float rotation_angle_radian =  rotation * (M_PI / 180.0);  
    float rotation_cosinus = cos( rotation_angle_radian) ;
    float rotation_sinus = sin( rotation_angle_radian) ; 

    bool minimum = false ;
    bool mid = false ; 
    bool max_bool = false ; 

    //find gradient 
    gradient = computeGradient(frontier) ; 
    current_pos = frontier ;
    // ROS_INFO(" Gradient is %f %f %f ",gradient.x(), gradient.y(), gradient.z());
    // ROS_INFO(" Position is %f %f %f ",frontier.x(), frontier.y(), frontier.z());
    if( (gradient.x() == 0) && (gradient.y() == 0) && (gradient.z() ==0) ){
        ROS_INFO(" Gradient is 0 : view can not be found");
        angle_diff = 0 ;
        return false; 
    }

    //very important ! 
    gradient.normalize() ;

    //change of angle if necessary 
    if( rotation != 0){

        //gradient.normalize() ;

        //Eigen::Matrix3d mat_rot = Eigen::Matrix3d::Zero();
        //mat(row, col) = value
        // mat_rot(0,0) = ( gradient.x() * gradient.x() ) * ( 1 - rotation_cosinus ) + rotation_cosinus ;
        // mat_rot(0,1) = ( gradient.x() * gradient.y() ) * ( 1 - rotation_cosinus ) - ( gradient.z() * rotation_sinus ) ;
        // mat_rot(0,2) = ( gradient.x() * gradient.z() ) * ( 1 - rotation_cosinus ) + ( gradient.y() * rotation_sinus ) ;
        
        // mat_rot(1,0) = ( gradient.x() * gradient.y() ) * ( 1 - rotation_cosinus ) + ( gradient.z() * rotation_sinus ) ;
        // mat_rot(1,1) = ( gradient.y() * gradient.y() ) * ( 1 - rotation_cosinus ) + rotation_cosinus ;
        // mat_rot(1,2) = ( gradient.y() * gradient.z() ) * ( 1 - rotation_cosinus ) - ( gradient.x() * rotation_sinus ) ;

        // mat_rot(2,0) = ( gradient.x() * gradient.z() ) * ( 1 - rotation_cosinus ) - ( gradient.y() * rotation_sinus ) ;
        // mat_rot(2,1) = ( gradient.y() * gradient.z() ) * ( 1 - rotation_cosinus ) + ( gradient.x() * rotation_sinus ) ;
        // mat_rot(2,2) = ( gradient.z() * gradient.z() ) * ( 1 - rotation_cosinus ) + rotation_cosinus ;
        
        // ROS_INFO(" Rotation goal is %f ", rotation);
        // ROS_INFO(" Rotation goal gradient is  %f ", rotation_angle_radian);

        Eigen::Vector3d temp_vect_calc( gradient.x(), gradient.y(), gradient.z() + 1) ; 
        Eigen::Vector3d axis_rotation = gradient.cross( temp_vect_calc);
        axis_rotation.normalize();

        Eigen::Matrix3d mat_rot = Eigen::AngleAxisd(rotation_angle_radian, axis_rotation).toRotationMatrix();

        

        vector_director = mat_rot * gradient ; 
        // ROS_INFO(" Rotated gradient is %f %f %f ",vector_director.x(), vector_director.y(), vector_director.z());


    }
    else{
        vector_director = gradient ; 
    }



    //ANGLE VERIFICATION
    ROS_INFO(" Angle should be between %f and %f", m_angle_low, m_angle_high);

    float vertical_view_angle = 0 ; 
    Eigen::Vector3d test_point = current_pos + 10 * vector_director; 
    ViewCandidate temp_view = { test_point.x() , test_point.y() , test_point.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
    findOrientation(temp_view) ; 
    verify_angle( frontier, temp_view, vertical_view_angle ) ; 
    ROS_INFO(" Original angle found is %f", vertical_view_angle);

    if( vertical_view_angle > m_angle_high ){
        angle_diff = m_angle_high - angle_mid ;
        rejected_view_candidates.push_back(temp_view);
        ROS_INFO(" Angle rejected :  %f ! Too high ! ", vertical_view_angle);
        return false;
    }
    else if( vertical_view_angle < m_angle_low ){
        angle_diff = m_angle_low - angle_mid ;
        rejected_view_candidates.push_back(temp_view);
        ROS_INFO(" Angle rejected :  %f ! Too low ! ", vertical_view_angle);
        return false;
    }
    else {
        angle_diff = 0;
        ROS_INFO(" Angle accepted :  %f ! ", vertical_view_angle);
    }


    //TO DO : test the uncommented line
    //current_pos = current_pos + ( (m_distance_min / m_map.getVoxelSize() ) -1 ) * vector_director ; 

    //int number_it = (m_distance_max + 0.2 - m_distance_min) / ( m_map.getVoxelSize()) ; //UNSURE! !!!!!!!!!!!!
    int number_it = (m_distance_max + 0.2) / (vector_director.norm() * m_map.getVoxelSize()) ; 
    //go along the gradient direction
    for(int k =0 ; k< number_it; k++){
    //while( distance < m_distance_max){

        current_pos = current_pos + (1 * vector_director) ;  
        distance = ((current_pos - frontier).norm()) * m_map.getVoxelSize(); 
        // ROS_INFO(" Distance is %f  until %f", distance, m_distance_max);

        
        //if we are in the correct range 
        //compute is position safe 
        if(distance > (m_distance_min - 0.2) ){ //start computing safe positions just before zone of interest
            isSafeView_bool = isSafeView(current_pos); 
            //isSafeView_bool = true; 
        }

        //closest point possible 
        if ((minimum == false) && (distance >  m_distance_min) && (distance < (m_distance_min + m_distance_max)/2 ) && isSafeView_bool) {
            min_view = { current_pos.x() , current_pos.y() , current_pos.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
            // ROS_INFO(" View found min");
            minimum = true ; 
        }
        //closest point from the center
        else if( (mid == false) && (distance < m_distance_max) && (distance > (m_distance_min + m_distance_max)/2 ) && isSafeView_bool ){
            mid_view = { current_pos.x() , current_pos.y() , current_pos.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
            // ROS_INFO(" View found mid");
            mid = true ; 
        }


    }
    if( isSafeView_bool ){
        max_view = { current_pos.x() , current_pos.y() , current_pos.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
        // ROS_INFO(" View found max");
        max_bool = true ; 
    }

    // ROS_INFO(" Angle should be between %f and %f", m_angle_low, m_angle_high);
    //We prioritize the value in the middle
    if(mid){
        findOrientation(mid_view) ; 
        isAngleok = verify_angle( frontier, mid_view , angle_mid ) ; 
        ROS_INFO(" Angle found is %f", angle_mid);
        if( isAngleok ){
            // ROS_INFO(" Angle of mid candidate accepted  %f", angle_mid);
            // ROS_INFO(" mid view found %f %f %f", mid_view.x, mid_view.y, mid_view.z);
            vc= mid_view ; 
            return true;
        }
    }


    if(minimum){
        findOrientation(min_view) ; 
        isAngleok = verify_angle( frontier, min_view , angle_min ) ; 
        ROS_INFO(" Angle found is  %f", angle_min);
        if( isAngleok ){
            // ROS_INFO(" Angle of min candidate accepted  %f", angle_min);
            // ROS_INFO(" min view found %f %f %f", min_view.x, min_view.y, min_view.z);
            vc = min_view ; 
            return true;
        }
    }

    if (max_bool){
        findOrientation(max_view) ; 
        isAngleok = verify_angle( frontier, max_view , angle_max ) ; 
        ROS_INFO(" Angle found is  %f", angle_max);
        if( isAngleok ){
            // ROS_INFO(" Angle of max candidate accepted  %f", angle_max);
            // ROS_INFO(" max view found %f %f %f", max_view.x, max_view.y, max_view.z);
            vc = max_view  ; 
            return true;

        }

    }

    // ROS_INFO("No position found : angle mid is  %f", angle_mid);
    // if( angle_mid > m_angle_high ){
    //     angle_diff = m_angle_high - angle_mid ;
    // }
    // else if( angle_mid < m_angle_low ){
    //     angle_diff = m_angle_low - angle_mid ;
    // }
    // else {
    //     angle_diff = angle_mid;

    // }

    if( angle_diff == 0 ){
        rejected_view = { current_pos.x() , current_pos.y() , current_pos.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
        rejected_view_candidates.push_back(rejected_view);
        ROS_INFO(" Angle correct but could not find any view ");

    }

    return false;




}

bool ViewGenerator::verify_angle(const Eigen::Vector3d& frontier, const ViewCandidate& vc, float& angle){

    //if find orientation failed to find a direction
    if( vc.q_x == 0 &&  vc.q_y == 0 && vc.q_z == 0 && vc.q_w == 0 ){
        return false;
    }

    double vertical_angle_rad = 0 ;
    double vertical_angle_deg = 0 ;
    Eigen::Vector3d next_voxel( vc.x, vc.y, vc.z );

    //Compute direction with frontier
    Eigen::Vector3d direction_original = (next_voxel - frontier).normalized() ; // from view candidate, towards frontier
    Eigen::Vector3d direction_proj( direction_original.x(), direction_original.y(), 0) ;
    direction_proj.normalize() ;
    
    //Compute signed vertical angle
    //vertical_angle_rad = std::atan2(direction_original.z(), direction_original.head<2>().norm());
    vertical_angle_rad = std::atan2(direction_original.z(), direction_proj.norm()); //same as last line
    vertical_angle_deg = vertical_angle_rad * (180.0 / M_PI);

    // ROS_INFO("Angle found is function is  %f", vertical_angle_deg);
    angle = vertical_angle_deg;
    if( (vertical_angle_deg < m_angle_high ) && (vertical_angle_deg > m_angle_low) ){
        angle = vertical_angle_deg;
        return true;
    }

    return false;

}



bool ViewGenerator::isSafeView(const Eigen::Vector3d& voxel){

    if( isCorrectPos(voxel) == false ){
        return false;
    }

    char state ;    
    state = m_map.getVoxelState_ESDF(voxel) ;
    float distance = 0 ;
    // ROS_INFO("state of voxel (%f %f %f) in the ESDF is  '%c'", voxel.x(), voxel.y(), voxel.z(), state ) ;
    if( state == voxblox_map::VoxbloxMap::OCCUPIED ){
        return false;

    }
    int min_radius = static_cast<int>(std::floor(-robot_radius_/2)) ; 
    int max_radius = static_cast<int>(std::ceil(robot_radius_/2)) ;
    for(int i= min_radius ; i <= max_radius ; i++){
        for(int j= min_radius ; j <= max_radius ; j++){
            for(int k= min_radius ; k <= max_radius ; k++){

                Eigen::Vector3d shift = Eigen::Vector3d(i,j,k);
                state = m_map.getVoxelState_ESDF(voxel + shift) ;
                distance = m_map.getVoxelDistance_ESDF(voxel + shift) ;
                //state = m_map.getVoxelState_TSDF(voxel + shift, 0);
                // ROS_INFO("state of voxel (%f %f %f) in the ESDF is  '%d'", (voxel+shift).x(), (voxel+shift).y(), (voxel+shift).z(),  static_cast<int>(state) ) ;
                //if ( distance <= 1 ){
                if( state == voxblox_map::VoxbloxMap::OCCUPIED ){
                    //ROS_INFO("FALSE");
                    return false;
                }

            }
        }
    }
    // ROS_INFO("TRUE");
    return true;

}

bool ViewGenerator::isCorrectPos(const Eigen::Vector3d& pos){
    // float x = pos.x();
    // float y = pos.y();
    // float z = pos z.();
    if(pos.z() < 3 ){
        return false; 
    }

    if( pos.z() < 0 ){
        return false;
    }
    return true ; 

  
}


// bool ViewGenerator::isSafeView(const Eigen::Vector3d& voxel){
//     //isFree = (current_state == voxblox_map::VoxbloxMap::FREE);
//     //const voxblox::EsdfVoxel& voxel = 
//     float dist = m_map.getVoxelDistance_ESDF(voxel) ;
//     if( dist < (robot_radius_ * m_map.getVoxelSize() * 0.5 ) ){
//         return false;
//     }
//     for(int i= -std::ceil(robot_radius_/2) ; i < std::ceil(robot_radius_/2) ; i++){
//         for(int j= -std::ceil(robot_radius_/2) ; i < std::ceil(robot_radius_/2) ; i++){
//             for(int k= -std::ceil(robot_radius_/2) ; i < std::ceil(robot_radius_/2) ; k++){

//                 Eigen::Vector3d shift = Eigen::Vector3d(i,j,k);
//                 dist = m_map.getVoxelDistance_ESDF(voxel + shift) ;
//                 if( dist < m_map.getVoxelSize() ){
//                     return false;
//                 }

//             }
//         }
//     }
//     return true;

// }



Eigen::Vector3d ViewGenerator::computeGradient(const Eigen::Vector3d& voxel){
    

    float vs = m_map.getVoxelSize() ; 
    Eigen::Vector3d gradient_vector(0,0,0); 
    Eigen::Vector3d shift_x(vs,0,0) ; 
    Eigen::Vector3d shift_y(0,vs,0) ; 
    Eigen::Vector3d shift_z(0,0,vs) ; 

    // gradient_vector.x() = m_map.getVoxelDistance_ESDF( voxel + shift_x) - m_map.getVoxelDistance_ESDF( voxel - shift_x);
    // gradient_vector.y() = m_map.getVoxelDistance_ESDF( voxel + shift_y) - m_map.getVoxelDistance_ESDF( voxel - shift_y);
    // gradient_vector.z() = m_map.getVoxelDistance_ESDF( voxel + shift_z) - m_map.getVoxelDistance_ESDF( voxel - shift_z);

    // ROS_INFO("List voxels analyzed");

    // Eigen::Vector3d center;
    // if (m_map.getVoxelCenter_ESDF(&center, voxel)) {
    //     ROS_INFO("%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel + shift_x)) {
    //     ROS_INFO("%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel - shift_x)) {
    //     ROS_INFO("%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel + shift_y)) {
    //     ROS_INFO("%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel - shift_y)) {
    //     ROS_INFO("%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel + shift_z)) {
    //     ROS_INFO("%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel - shift_z)) {
    //     ROS_INFO("%f %f %f", center.x(), center.y(), center.z());
    // }
    
    // ROS_INFO("End list");


    
    

    gradient_vector.x() = m_map.getVoxelDistance_ESDF( voxel + shift_x) - m_map.getVoxelDistance_ESDF( voxel - shift_x);
    gradient_vector.y() = m_map.getVoxelDistance_ESDF( voxel + shift_y) - m_map.getVoxelDistance_ESDF( voxel - shift_y);
    gradient_vector.z() = m_map.getVoxelDistance_ESDF( voxel + shift_z) - m_map.getVoxelDistance_ESDF( voxel - shift_z);

    gradient_vector.x() = gradient_vector.x() / (2 *vs) ; 
    gradient_vector.y() = gradient_vector.y() / (2*vs) ;
    gradient_vector.z() = gradient_vector.z() / (2*vs);

    if (gradient_vector.norm() == 0){
        //if gradient = 0 use forward difference
        // ROS_INFO("Forward difference");


        gradient_vector.x() = ( m_map.getVoxelDistance_ESDF( voxel + shift_x) - m_map.getVoxelDistance_ESDF( voxel ) ) / vs ;
        gradient_vector.y() = ( m_map.getVoxelDistance_ESDF( voxel + shift_y) - m_map.getVoxelDistance_ESDF( voxel ) ) / vs ;
        gradient_vector.z() = (m_map.getVoxelDistance_ESDF( voxel + shift_z) - m_map.getVoxelDistance_ESDF( voxel ) ) /vs ;

    }

    if (gradient_vector.norm() == 0){
        // ROS_INFO("Gradient hardouin");

        //if gradient = 0 use hardouin weighted based method 

        return computeGradient_26(voxel); 

    }
    
    


    //Correction

    // if( gradient_vector.x() > 0){
    //     gradient_vector.x() = std::ceil( gradient_vector.x() ) ;
    // }
    // else{
    //     gradient_vector.x() = std::floor( gradient_vector.x() ) ;
    // }

    // if( gradient_vector.y() > 0){
    //     gradient_vector.y() = std::ceil( gradient_vector.y() ) ;
    // }
    // else{
    //     gradient_vector.y() = std::floor( gradient_vector.y() ) ;
    // }

    // if( gradient_vector.z() > 0){
    //     gradient_vector.z() = std::ceil( gradient_vector.z() ) ;
    // }
    // else{
    //     gradient_vector.z() = std::floor( gradient_vector.z() ) ;
    // }

    return gradient_vector;

}


//Hardouin method 
Eigen::Vector3d ViewGenerator::computeGradient_26(const Eigen::Vector3d& voxel){
    // ROS_INFO("Gradient hardouin");


    Eigen::Vector3d grad_dir ; 
    Eigen::Vector3d temp_dir ; 
    double weight = 0 ; 
    for(int i = 0; i< 26;i++){
        
        weight = m_map.getVoxelWeight_TSDF( voxel + c_neighbor_voxels_[i] ) ; 
        if( weight > 0 && m_map.getVoxelDistance_TSDF( voxel + c_neighbor_voxels_[i] ) < m_map.getVoxelSize() ){
            weight = -weight ; 
        }
        temp_dir = c_neighbor_voxels_[i].normalized() ; 
        grad_dir = grad_dir + (weight * temp_dir) ; 
        

    }

    grad_dir.normalize();

    return grad_dir ; 


}

Eigen::Vector3d ViewGenerator::computeGradient_Sobel(const Eigen::Vector3d& voxel){
    Eigen::Vector3d vect;
    return vect;
}

// Eigen::Vector3d ViewGenerator::computeGradient2(const Eigen::Vector3d& voxel){
    

//     float vs = 1 ; 
//     Eigen::Vector3d gradient_vector(0,0,0); 
//     Eigen::Vector3d shift_x(vs,0,0) ; 
//     Eigen::Vector3d shift_y(0,vs,0) ; 
//     Eigen::Vector3d shift_z(0,0,vs) ; 
    



// }


// ViewGenerator::ViewGenerator(/* args */)
// {
// }

// ViewGenerator::~ViewGenerator()
// {
// }
