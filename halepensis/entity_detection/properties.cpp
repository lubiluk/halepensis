#include "properties.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#pragma clang diagnostic pop

Properties::Properties(Vector position,
                       Rotation rotation,
                       Vector mass_center):
position(position),
rotation(rotation),
mass_center(mass_center)
{
    
}

auto detect_properties(const std::shared_ptr<PointCloud>& cloud) -> Properties
{
    pcl::MomentOfInertiaEstimation <Point> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute();
    
//    std::vector <float> moment_of_inertia;
//    std::vector <float> eccentricity;
    Point min_point_AABB;
    Point max_point_AABB;
    Point min_point_OBB;
    Point max_point_OBB;
    Point position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
//    float major_value, middle_value, minor_value;
//    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    
//    feature_extractor.getMomentOfInertia (moment_of_inertia);
//    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
//    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
//    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);
    
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Rotation rot (rotational_matrix_OBB);
    
    return Properties(position, rot, mass_center);
}
