#pragma once

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "planning/data/type_conversions.h"

namespace map_frontiers { namespace visualization {

visualization_msgs::Marker CreateHorizontalFOVMarker(const geometry_msgs::Pose& view, int markerId, float range) {
    // Define the Marker message
    visualization_msgs::Marker fovMarker;
    fovMarker.header.frame_id = "world";  // Reference frame
    fovMarker.header.stamp = ros::Time::now();
    fovMarker.ns = "horizontal_fov";
    fovMarker.id = markerId;
    fovMarker.type = visualization_msgs::Marker::CYLINDER;
    fovMarker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker (position and orientation) and its scales
    fovMarker.pose = view;
    fovMarker.scale.x = range;  // radius of the cylinder
    fovMarker.scale.y = range;  // radius of the cylinder
    fovMarker.scale.z = 0.05;  // height of the cylinder

    // Set the color (RGBA)
    fovMarker.color.r = 1.0f;
    fovMarker.color.g = 0.0f;
    fovMarker.color.b = 0.0f;
    fovMarker.color.a = 0.6f;

    return fovMarker;
    }

visualization_msgs::Marker CreateVerticalFOVMarker(const geometry_msgs::Pose& view, int markerId, float range) {
    // Define the Marker message
    visualization_msgs::Marker fovMarker;
    fovMarker.header.frame_id = "world";  // Reference frame
    fovMarker.header.stamp = ros::Time::now();
    fovMarker.ns = "vertical_fov";
    fovMarker.id = markerId;
    fovMarker.type = visualization_msgs::Marker::LINE_STRIP;  // Use LINE_STRIP to draw lines
    fovMarker.action = visualization_msgs::Marker::ADD;

    // Set the color and scale of the triangle
    fovMarker.color.r = 1.0;
    fovMarker.color.g = 0.0;
    fovMarker.color.b = 0.0;
    fovMarker.color.a = 0.5;  // Semi-transparent red

    fovMarker.scale.x = 0.05;  // Thickness of the lines

    // Set the range and FOV angle
    float angleFovUp = 57.0 * M_PI / 180.0;  // Half of the vertical FOV, in rad
    float angleFovDown = 25.0 * M_PI / 180.0;  // Half of the vertical FOV, in rad

    // Define the points for the vertical fov triangle in the local drone frame
    float topOffsetInZ = range / sin(angleFovDown);
    float downOffsetInZ = range / sin(angleFovUp);
    tf2::Vector3 topPointLocal(range, 0.0, angleFovUp);
    tf2::Vector3 downPointLocal(range, 0.0, -angleFovDown);
    
    // Project them in world frame
    tf2::Transform worldToDroneTransform;
    conversions::GeometryPoseToTf2Transform(view, worldToDroneTransform);
    tf2::Vector3 topPointGlobal = worldToDroneTransform * topPointLocal;
    tf2::Vector3 downPointGlobal = worldToDroneTransform * downPointLocal; 

    // Add them to the marker's point list
    geometry_msgs::Point topGeometryPointGlobal, downGeometryPointGlobal;
    conversions::tf2VectorToGeometryPoint(topPointGlobal, topGeometryPointGlobal);
    conversions::tf2VectorToGeometryPoint(downPointGlobal, downGeometryPointGlobal);

    fovMarker.points.push_back(view.position);
    fovMarker.points.push_back(topGeometryPointGlobal);
    fovMarker.points.push_back(downGeometryPointGlobal);
    fovMarker.points.push_back(view.position);
    return fovMarker;
}

} //namespace visualization

} //namespace map_frontiers
