#pragma once

#include <ros/ros.h>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include <planning/modules/utils.h>

class NBVSelectorParameters {
private:
    std::unordered_set<std::string> _views_generation_methods = {"gradient"};

public:
    /////////////// GENERAL
    float quality_objective;  // Coefficient between 0 and 1. TODO: description!
    bool use_freespace;

    /////////////// NAVIGATION
    float tolerance_distance; // In m. Max acceptable distance between current pose and goal for it to be seen as reached
    float angular_tolerance;  // In degrees. Max acceptable angle between current pose and goal for it to be seen as reached
    double threshold_known;  // from Hardouin // NOTE: Is it used?

    ////////////// Planning parameters
    float image_component_weight; // TODO: description
    float metrics_component_weight; // TODO: description
    bool use_distance_in_metrics_component;  // TODO: description
    bool do_occlusions_check; // TODO: description

    ////////////// View Generation parameters
    std::string views_generation_method; // TODO: description
    float distance_min; // In m. Min distance between the frontier point and the generated view.
    float distance_max; // In m. Max distance between the frontier point and the generated view.
    int subsampling_views; // Number of subsampled views. The bigger the faster but the less likely to find a NBV.
    float robot_radius; // Max number of voxels occupied by the robots in one direction : if 5, robot is contained in a 5*5*5 voxel cube
    int radius_surface_max; // In m. zone around the frontier to search for surface voxels
    float angle_low; // In degrees. Vertical angle below the drone 
    float angle_high; // In degrees. Vertical angle above the drone 

    ////////////// View Evaluator parameters
    float value_frontier;  // NOTE: Is it used?
    std::string views_selection_method; 

    ////////////// View Evaluator parameters
    std::string views_sampling_method; 

    ////////////// LIDAR Parameters
    // TODO: should be only in SensorModel
    Eigen::Vector3d mounting_translation_;  // x,y,z [m] // NOTE: Is it used?
    Eigen::Quaterniond mounting_rotation_;  // x,y,z,w quaternion  // NOTE: Is it used?
    //sensor parameters
    double p_ray_length;  // In m. Expected max sensor ray length
    double p_fov_x;  // In degrees. Total fields of view, expected symmetric w.r.t.
    // sensor facing direction
    double p_fov_y;  // In degrees. Total fields of view, expected symmetric w.r.t.
    int p_resolution_x; // In pixel. Expected resolution of the sensor
    int p_resolution_y; // In pixel. Expected resolution of the sensor
    double p_sampling_time; // In s. 


    ////////////// Bounding Box
    // In m. Max/min value that will define an exploration bounding box in which the views
    // and next best views can be computed
    BoundingBox bounding_box; // {xmax, xmin, ymax, ymin, zmax, zmin}

    ////////////// Logs
    bool verbose;
    bool timer;
    bool extra_viz;

    // Constructor
    NBVSelectorParameters();

    void SetDefaultValues();

    void LoadFromRos(const ros::NodeHandle& nh);

    void CheckValues();
};


NBVSelectorParameters::NBVSelectorParameters(){
    SetDefaultValues();
}

void NBVSelectorParameters::SetDefaultValues(){
    quality_objective = 1.0;
    use_freespace = false;

    tolerance_distance = 0.2;
    angular_tolerance = 10.0 * M_PI / 180.0;
    threshold_known = 0.0;

    image_component_weight = 1.0;
    metrics_component_weight = 1.0;
    use_distance_in_metrics_component = true; 
    do_occlusions_check = false;

    views_generation_method = "gradient";
    distance_min = 3.0;
    distance_max = 4.0;
    subsampling_views = 100;
    robot_radius = 8;
    radius_surface_max = 3;
    angle_low = -25.0;
    angle_high = 57.0;

    mounting_translation_ = Eigen::Vector3d();
    mounting_rotation_ = Eigen::Quaterniond();
    p_ray_length = 10.0;
    p_fov_x = 360.0 * M_PI / 180.0;
    p_fov_y = 63.05 * M_PI / 180.0;
    p_resolution_x = 1000;
    p_resolution_y = 1000;
    p_sampling_time = 1.0;

    // current value is for HOUSE env. 
    bounding_box = {10.0, -20.0, 11.0, -11.0, 100.0, 0.0};
}

void NBVSelectorParameters::LoadFromRos(const ros::NodeHandle& nh){
    nh.param("NBV_selector/sub_sample_size", subsampling_views, subsampling_views);
    ROS_INFO("Received sub_sample_size: %i", subsampling_views);

    nh.param("NBV_selector/angle_low", angle_low, angle_low);
    ROS_INFO("Received angle_low: %f", angle_low);

    nh.param("NBV_selector/angle_high", angle_high, angle_high);
    ROS_INFO("Received angle_high: %f", angle_high);

    nh.param("NBV_selector/p_ray_length", p_ray_length, p_ray_length);
    ROS_INFO("Received p_ray_length: %f", p_ray_length);

    nh.param("NBV_selector/quality_objective", quality_objective, quality_objective);
    ROS_INFO("Received image qualitive objective: %f", quality_objective);

    nh.param("NBV_selector/image_component_weight", image_component_weight, image_component_weight);
    ROS_INFO("Received image component coefficient: %f", image_component_weight);

    nh.param("NBV_selector/metrics_component_weight", metrics_component_weight, metrics_component_weight);
    ROS_INFO("Received navigation component coefficient: %f", metrics_component_weight);

    nh.param("NBV_selector/use_distance_in_metrics_component", use_distance_in_metrics_component, use_distance_in_metrics_component);
    if (use_distance_in_metrics_component) {
      ROS_INFO("Using angular and linear distances to compute the cost weight of each view.");
    } else {
      ROS_INFO("Using angular distance only to compute the cost weight of each view.");
    }

    nh.param("NBV_selector/do_occlusions_check", do_occlusions_check, do_occlusions_check);
    ROS_INFO("Doing occlusion check: %s", do_occlusions_check ? "true" : "false");

    nh.param("NBV_selector/use_freespace", use_freespace, use_freespace);
    ROS_INFO("Using freespace: %s", use_freespace ? "true" : "false");

    std::vector<double> bounding_box_vec;
    nh.param("NBV_selector/bounding_box", bounding_box_vec, bounding_box_vec);
    if (bounding_box_vec.size() == 6) {
        bounding_box = {bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]};
        ROS_INFO("Received bounding_box: {%f, %f, %f, %f, %f, %f}", bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]);
    } else {
        ROS_WARN("Received a bounding box param from ros but with the wrong size. Please check your setup.");
    }

    nh.param("NBV_selector/views_generation_method", views_generation_method, views_generation_method);
    ROS_INFO("Received views_generation_method: %s", views_generation_method);

    nh.param("NBV_selector/timer", timer, timer);
    ROS_INFO("Enabling timer: %s", timer ? "true" : "false");

    nh.param("NBV_selector/verbose", verbose, verbose);
    ROS_INFO("Enabling verbose: %s", verbose ? "true" : "false");

    nh.param("NBV_selector/extra_viz", extra_viz, extra_viz);
    ROS_INFO("Enabling extra_viz: %s", extra_viz ? "true" : "false");

    CheckValues();
}

void NBVSelectorParameters::CheckValues(){
    // Check that weights are between 0 and 1
    if (image_component_weight < 0.0 || image_component_weight > 1.0) {
        throw std::runtime_error("Parameter 'image_component_weight' must be between 0.0 and 1.0");
    }
    if (metrics_component_weight < 0.0 || metrics_component_weight > 1.0) {
        throw std::runtime_error("Parameter 'metrics_component_weight' must be between 0.0 and 1.0");
    }
    if (quality_objective < 0.0 || quality_objective > 1.0) {
        throw std::runtime_error("Parameter 'quality_objective' must be between 0.0 and 1.0");
    }

    // Check that the methods' name exist
    if (_views_generation_methods.find(views_generation_method) == _views_generation_methods.end()) {
        throw std::runtime_error("Parameter 'views_generation_method' does not exist in the list of possible method. Please check your setup.");
    }
}