#ifndef VOXBLOX_MAP_VOXBLOX_MAP_H_
#define VOXBLOX_MAP_VOXBLOX_MAP_H_

#include <memory>
#include <Eigen/Eigen>
#include <iostream>
#include <string>
//#include <voxblox_ros/tsdf_server.h>
//#include <voxblox_ros/voxfield_server.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"


typedef float FloatingPoint;

namespace voxblox_map {

//BIG NOTE : TRY TO CONST AS MUCH AS POSSIBLE METHODS
//TO CHANGE GET DISTANCE AND GET WEIGHT BOTH RETURN  IF VOXEL DOES NOT EXIST

// Voxblox as a map representation
class VoxbloxMap {
 public:
  VoxbloxMap(){};
  explicit VoxbloxMap(FloatingPoint voxel_size, size_t voxels_per_side);  // NOLINT // !!! : collision radius is unique here. It should be unique for each robot, to correct // TO CHANGE

  // check collision for a single pose
  bool isTraversable_ESDF(const Eigen::Vector3d& position,
                     const Eigen::Quaterniond& orientation) ;

  // check whether point is part of the map
  bool isObserved_ESDF(const Eigen::Vector3d& point) ;

  // get occupancy
  unsigned char getVoxelState_ESDF(const Eigen::Vector3d& point) ;

  // get voxel size
  double getVoxelSize() ;

  // get the center of a voxel from input point
  bool getVoxelCenter_ESDF(Eigen::Vector3d* center,
                      const Eigen::Vector3d& point) ;
                      
  bool getVoxelCenter_TSDF(Eigen::Vector3d* center,
                      const Eigen::Vector3d& point) ;

  // get the stored distance
  double getVoxelDistance_TSDF(const Eigen::Vector3d& point) ;

  // get the stored weight
  double getVoxelWeight_TSDF(const Eigen::Vector3d& point) ;

  // get the maximum allowed weight (return 0 if using uncapped weights)
  double getMaximumWeight() ;

    //states
  const static unsigned char OCCUPIED = 0;  // NOLINT
  const static unsigned char FREE = 1;      // NOLINT
  const static unsigned char UNKNOWN = 2;   // NOLINT
  const static unsigned char UNSURE_OCC = 3; // NOLINT
  const static unsigned char UNSURE_FREE = 4; // NOLINT 
  
  
  std::shared_ptr<voxblox::EsdfMap> get_esdf_map_pointer(){ return esdf_map_pointer ; }
  std::shared_ptr<voxblox::TsdfMap> get_tsdf_map_pointer(){ return tsdf_map_pointer ; }
  double get_voxel_size();
  double get_block_size();
  double get_maximum_weight();
  double get_collision_radius();
  double get_confidence_threshold();
  double get_distance_threshold();

  
private:
  // esdf server that contains the map, subscribe to external ESDF/TSDF updates
  std::shared_ptr<voxblox::EsdfMap> esdf_map_pointer ; // std::shared_ptr<const EsdfMap>
  std::shared_ptr<voxblox::TsdfMap> tsdf_map_pointer ; // std::shared_ptr<const TsdfMap>

  std::shared_ptr< voxblox::Layer<voxblox::TsdfVoxel> > layer_tsdf_ ;
  std::shared_ptr< voxblox::Layer<voxblox::EsdfVoxel> > layer_esdf_ ;

  // cache constants
  double c_voxel_size_;
  double c_block_size_;
  double c_maximum_weight_;

  //my own values
  double m_collision_radius;
  
  //occupation values
  float m_confidence_threshold = 0;
  int m_distance_threshold ;

};

}  // namespace map

#endif  // VOXBLOX_MAP_VOXBLOX_MAP_H_