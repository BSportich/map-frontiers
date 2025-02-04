#include <voxblox_map/voxblox_map.h>

//#include "active_3d_planning_core/data/system_constraints.h"

namespace voxblox_map {



// VoxbloxMap::VoxbloxMap(int collision_radius) : {
//     // create an esdf server
//   ros::NodeHandle nh("");
//   ros::NodeHandle nh_private("~");
//   esdf_server_.reset(new voxblox::EsdfServer(nh, nh_private));
//   m_collision_radius = collision_radius;
//   esdf_server_->setTraversabilityRadius(
//       collision_radius);

//   // cache constants
//   c_voxel_size_ = esdf_server_->getEsdfMapPtr()->voxel_size();
//   c_block_size_ = esdf_server_->getEsdfMapPtr()->block_size();
//   c_maximum_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(nh_private)
//                           .max_weight;  // direct access is not exposed
// }


VoxbloxMap::VoxbloxMap(FloatingPoint voxel_size, size_t voxels_per_side) {

    layer_tsdf_.reset(new voxblox::Layer<voxblox::TsdfVoxel>(voxel_size, voxels_per_side));
    layer_esdf_.reset(new voxblox::Layer<voxblox::EsdfVoxel>(voxel_size, voxels_per_side));

    esdf_map_pointer = nullptr;
    tsdf_map_pointer = nullptr;

    esdf_map_pointer.reset(new voxblox::EsdfMap(layer_esdf_));
    tsdf_map_pointer.reset(new voxblox::TsdfMap(layer_tsdf_));

    m_distance_threshold = 0;
    m_confidence_threshold = 0 ;
    c_voxel_size_ = voxel_size;
    c_block_size_ = voxels_per_side;

}

// voxblox::EsdfServer& VoxbloxMap::getESDFServer() { return *esdf_server_; }


//use ESDF
bool VoxbloxMap::isTraversable_ESDF(const Eigen::Vector3d& position,
                               const Eigen::Quaterniond& orientation) {
  double distance = 0.0;
  if (esdf_map_pointer->getDistanceAtPosition(position,
                                                           &distance)) {
    // This means the voxel is observed
    return (distance > m_collision_radius); //CONSIDERS UNIQUE RADIUS !!! TO CHANGE
  }
  return false;
}
//getDisanceAtPosition unavailable in TSDF_map.h


bool VoxbloxMap::isObserved_ESDF(const Eigen::Vector3d& point) {
  return esdf_map_pointer->isObserved(point);
}
// isObserved is unavailable in TSDF_map.h


// get occupancy - use ESDF
//inappropriate !!! 
unsigned char VoxbloxMap::getVoxelState_ESDF(const Eigen::Vector3d& point) {
  double distance = 0.0;
  if (esdf_map_pointer->getDistanceAtPosition(point, &distance)) {
    // This means the voxel is observed
    if (distance < c_voxel_size_) {
      return VoxbloxMap::OCCUPIED;
    } else {
      return VoxbloxMap::FREE;
    }
  } else {
    return VoxbloxMap::UNKNOWN;
  }
}
// get occupancy - use TSDF
//getDistanceAtPosition unavailable in TSDF_map.h

//THIS FUNCTION SHOULD BE USED FOR FRONTIERS DETECTION !! 
unsigned char VoxbloxMap::getVoxelState_TSDF(const Eigen::Vector3d& point) {
  double distance = getVoxelDistance_TSDF(point);
  double weight = getVoxelWeight_TSDF(point);
  double threshold = 0.3 ; 
  if (weight > threshold) {
    // This means the voxel is observed
    if (distance < c_voxel_size_) {
      return VoxbloxMap::OCCUPIED;
    } else {
      return VoxbloxMap::FREE;
    }
  } else {
    return VoxbloxMap::UNKNOWN;
  }
}


// get voxel size 
double VoxbloxMap::getVoxelSize() { return c_voxel_size_; }

// get the center of a voxel from input point - use ESDF
bool VoxbloxMap::getVoxelCenter_ESDF(Eigen::Vector3d* center,
                                const Eigen::Vector3d& point) {
  voxblox::BlockIndex block_id = esdf_map_pointer
                                     ->getEsdfLayerPtr()
                                     ->computeBlockIndexFromCoordinates(
                                         point.cast<voxblox::FloatingPoint>());
  *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_)
                .cast<double>();
  voxblox::VoxelIndex voxel_id =
      voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
          (point - *center).cast<voxblox::FloatingPoint>(),
          1.0 / c_voxel_size_);
  *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_)
                 .cast<double>();
  return true;
}

// get the center of a voxel from input point - use TSDF
bool VoxbloxMap::getVoxelCenter_TSDF(Eigen::Vector3d* center,
                                const Eigen::Vector3d& point) {
  voxblox::BlockIndex block_id = tsdf_map_pointer
                                     ->getTsdfLayerPtr()
                                     ->computeBlockIndexFromCoordinates(
                                         point.cast<voxblox::FloatingPoint>());
  *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_)
                .cast<double>();
  voxblox::VoxelIndex voxel_id =
      voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
          (point - *center).cast<voxblox::FloatingPoint>(),
          1.0 / c_voxel_size_);
  *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_)
                 .cast<double>();
  return true;
}

// get the stored TSDF distance - use TSDF
double VoxbloxMap::getVoxelDistance_TSDF(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
      tsdf_map_pointer
          ->getTsdfLayerPtr()
          ->getBlockPtrByCoordinates(voxblox_point);
  if (block) {
    voxblox::TsdfVoxel* tsdf_voxel =
        block->getVoxelPtrByCoordinates(voxblox_point);
    if (tsdf_voxel) {
      return tsdf_voxel->distance;
    }
  }
  return 0.0;
}

// get the stored weight - use TSDF
double VoxbloxMap::getVoxelWeight_TSDF(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
      tsdf_map_pointer
          ->getTsdfLayerPtr()
          ->getBlockPtrByCoordinates(voxblox_point);
  if (block) {
    voxblox::TsdfVoxel* tsdf_voxel =
        block->getVoxelPtrByCoordinates(voxblox_point);
    if (tsdf_voxel) {
      return tsdf_voxel->weight;
    }
  }
  return 0.0;
}

// get the maximum allowed weight (return 0 if using uncapped weights)
double VoxbloxMap::getMaximumWeight() { return c_maximum_weight_; }

//evaluate the whole produced map 
double VoxbloxMap::evaluation_TSDF(float truncationdist){
  double result = 0.0;
  int voxel_count = 0 ;

  voxblox::BlockIndexList blocks;
  tsdf_map_pointer->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = tsdf_map_pointer->getTsdfLayerPtr()->voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  for (const voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel>& block = tsdf_map_pointer->getTsdfLayerPtr()->getBlockByIndex(index);

    voxblox::Point origin = block.origin();

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
          ++linear_index) {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
      Eigen::Vector3d coord_3d = Eigen::Vector3d(coord.x(), coord.y(), coord.z());

      //Test if voxel is observed
      float voxeldistance = getVoxelDistance_TSDF(coord_3d);
      if(voxeldistance > 0 ){
        
        if( voxeldistance < truncationdist){
          //if closes to the surface sums its weights
          result = result + getVoxelWeight_TSDF(coord_3d);
          voxel_count = voxel_count + 1;
        }
      } 

    }

  }

  return result/voxel_count;

}


}