#include <Eigen/Eigen>
#include <iostream>
#include "ViewGenerator.h"
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <voxblox_map/voxblox_map.h>
#include <planning/NBV_Selector.cpp>


int main(){


    //create system constraints



    //create Voxfield map
    double collision_radius = 5;
    VoxbloxMap my_map = VoxbloxMap(5);


    //create NBV Selector
    ViewGenerator my_view_gen = ViewGenerator();
    NBV_Selector my_selector = NBV_Selector(my_map, my_view_gen);


    //Test
    my_selector.get

}