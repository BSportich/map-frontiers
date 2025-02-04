#pragma once
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <voxblox_map/voxblox_map.h>
#include <string>
#include <planning/modules/SensorModel.h>


class ViewEvaluator
{
private:
    /* data */
    std::string m_method_type;
    voxblox_map::VoxbloxMap m_map;

    // params
    double p_ray_step_;
    double p_downsampling_factor_;  // Artificially reduce the minimum resolution
    // to increase performance

    // constants
    int c_res_x_;  // factual resolution that is used for ray casting
    int c_res_y_;
    int c_n_sections_;  // number of ray duplications
    std::vector<double>
        c_split_distances_;            // distances where rays are duplicated
    std::vector<int> c_split_widths_;  // number of max distance rays that are
                                      // covered per split

    // variables
    Eigen::ArrayXXi ray_table_;

    //parameters for the view evaluation
    float value_frontier_ ; 

public:
    ViewEvaluator(const voxblox_map::VoxbloxMap& map, const std::string& method_name, , const SensorModel::SensorModel& sensor_lidar) ;
    ~ViewEvaluator();

    // float evaluate_view_image(const ViewCandidate& vc); 
    // float evaluate_view_pos(const ViewCandidate& vc, const ViewCandidate& current_pos);

    float evaluate_voxel_image(const Eigen::Vector3d point);
    float evaluate_view_image(const std::vector<Eigen::Vector3d>& voxels_set);

    float count_frontiers_view(const std::vector<Eigen::Vector3d>& voxels_set,const std::vector<Eigen::Vector3d>& frontiers_set);

    void getVisibleVoxels_camera(const ViewCandidate& vc);
    void getVisibleVoxels_LIDAR(const ViewCandidate& vc);
    void markNeighboringRays(int x, int y, int segment, int value);
    float count_frontiers_view(const std::vector<Eigen::Vector3d>& voxels_set,const std::vector<Eigen::Vector3d>& frontiers_set);

};

ViewEvaluator::ViewEvaluator(const voxblox_map::VoxbloxMap& map, const std::string& method_name, const SensorModel::SensorModel& sensor_lidar) 
{
  m_map = map ;
  m_method_type = method_name ;
  p_ray_step_ = m_map->getVoxelSize() ;
  p_downsampling_factor_ = 1.0 ;

  // Downsample to voxel size resolution at max range
  c_res_x_ = std::min(static_cast<int>(std::ceil(
                          sensor_lidar.p_ray_length_ * sensor_lidar.p_fov_x_ /
                          (m_map->getVoxelSize() * p_downsampling_factor_))),
                      sensor_lidar.p_resolution_x_);
  c_res_y_ = std::min(static_cast<int>(std::ceil(
                          sensor_lidar.p_ray_length_ * sensor_lidar.p_fov_y_ /
                          (m_map->getVoxelSize() * p_downsampling_factor_))),
                      sensor_lidar.p_resolution_y_);

  // Determine number of splits + split distances
  c_n_sections_ =
      std::floor(static_cast<double>(std::log2(std::min(c_res_x_, c_res_y_))));
  c_split_widths_.push_back(0);
  for (int i = 0; i < c_n_sections_; ++i) {
    c_split_widths_.push_back(std::pow(2, i));
    c_split_distances_.push_back(sensor_lidar.p_ray_length_ /
                                 std::pow(2.0, static_cast<double>(i)));
  }
  c_split_distances_.push_back(0.0);
  std::reverse(c_split_distances_.begin(), c_split_distances_.end());
  std::reverse(c_split_widths_.begin(), c_split_widths_.end());

}

// void ViewEvaluator::getVisibleVoxels_camera(const ViewCandidate& vc){
//   // Naive ray-casting
//   Eigen::Vector3d camera_direction;
//   Eigen::Vector3d direction;
//   Eigen::Vector3d current_position;
//   Eigen::Vector3d voxel_center;
//   for (int i = 0; i < c_res_x_; ++i) {
//     for (int j = 0; j < c_res_y_; ++j) {
//       CameraModel::getDirectionVector(
//           &camera_direction,
//           static_cast<double>(i) / (static_cast<double>(c_res_x_) - 1.0),
//           static_cast<double>(j) / (static_cast<double>(c_res_y_) - 1.0));
//       direction = orientation * camera_direction;
//       double distance = 0.0;
//       while (distance < p_ray_length_) {
//         current_position = position + distance * direction;
//         distance += p_ray_step_;

//         // Check voxel occupied
//         if (map_->getVoxelState(current_position) ==
//             map::OccupancyMap::OCCUPIED) {
//           break;
//         }

//         // Add point (duplicates are handled in
//         // CameraModel::getVisibleVoxelsFromTrajectory)
//         m_map->getVoxelCenter_TSDF(&voxel_center, current_position);
//         result->push_back(voxel_center);
//       }
//     }
//   }
//   return true

// }

void ViewEvaluator::getVisibleVoxels_LIDAR(
    std::vector<Eigen::Vector3d>* result, const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation) {
  // Setup ray table (contains at which segment to start, -1 if occluded
  ray_table_ = Eigen::ArrayXXi::Zero(c_res_x_, c_res_y_);

  // Ray-casting
  Eigen::Vector3d camera_direction;
  Eigen::Vector3d direction;
  Eigen::Vector3d current_position;
  Eigen::Vector3d voxel_center;
  double distance;
  bool cast_ray;
  double map_distance;
  for (int i = 0; i < c_res_x_; ++i) {
    for (int j = 0; j < c_res_y_; ++j) {
      int current_segment = ray_table_(i, j);  // get ray starting segment
      if (current_segment < 0) {
        continue;  // already occluded ray
      }
      LidarModel::getDirectionVector(
          &camera_direction,
          static_cast<double>(i) / (static_cast<double>(c_res_x_) - 1.0),
          static_cast<double>(j) / (static_cast<double>(c_res_y_) - 1.0));
      direction = orientation * camera_direction; // direction of the lidar in general * current orientation of the sensor
      distance = c_split_distances_[current_segment];
      cast_ray = true;
      while (cast_ray) {
        // iterate through all splits (segments)
        while (distance < c_split_distances_[current_segment + 1]) {
          current_position = position + distance * direction;
          distance += p_ray_step_;

          // Add point (duplicates are handled in
          // CameraModel::getVisibleVoxelsFromTrajectory)
          m_map->getVoxelCenter_TSDF(&voxel_center, current_position);
          result->push_back(voxel_center);

          // Check voxel occupied 
          if (m_map->getVoxelState_TSDF(current_position) ==
              VoxbloxMap::OCCUPIED) {
            // Occlusion, mark neighboring rays as occluded
            markNeighboringRays(i, j, current_segment, -1);
            cast_ray = false;
            break;
          }
        }
        if (cast_ray) {
          current_segment++;
          if (current_segment >= c_n_sections_) {
            cast_ray = false;  // done
          } else {
            // update ray starts of neighboring rays
            markNeighboringRays(i, j, current_segment - 1, current_segment);
          }
        }
      }
    }
  }
  return true;
}


void ViewEvaluator::markNeighboringRays(int x, int y, int segment,
                                                  int value) {
  // Set all nearby (towards bottom right) ray starts, depending on the segment
  // depth, to a value.
  for (int i = x; i < std::min(c_res_x_, x + c_split_widths_[segment]); ++i) {
    for (int j = y; j < std::min(c_res_y_, y + c_split_widths_[segment]); ++j) {
      ray_table_(i, j) = value;
    }
  }
}

//potential inefficiency : testing frontiers with dedicated method might be more efficient
float ViewEvaluator::count_frontiers_view(const std::vector<Eigen::Vector3d>& voxels_set,const std::vector<Eigen::Vector3d>& frontiers_set){
  float evaluation = 0 ;
  for(int i=0; i< voxels_set.size(); i++) {

    Eigen::Vector3d voxel_test = voxels_set[i];

    for(int j=0; j< frontiers_set.size(); j++){
        
        Eigen::Vector3d frontier = frontiers_set[j];

        if( voxel_test == frontier ){ // .isApprox() ?

          evaluation = evaluation + value_frontier_ ; 
        }


    }

  }

  return evaluation; 


}





ViewEvaluator::~ViewEvaluator()
{
}
