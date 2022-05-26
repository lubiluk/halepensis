#include "properties.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#pragma clang diagnostic pop

using Eigen::Affine3f;

properties::properties(vec3 position,
                       rot_mat rotation,
                       vec3 mass_center,
                       point min_corner,
                       point max_corner):
position(position),
rotation(rotation),
mass_center(mass_center),
min_corner(min_corner),
max_corner(max_corner)
{
    
}

auto properties::transformed(const mat44& transform) const -> properties
{
    Affine3f t{transform};
    auto t_position = t * position;
    auto t_rotation = t * rotation;
    auto t_mass_center = t * mass_center;
    auto t_min_corn = t * vec3{min_corner.x, min_corner.y, min_corner.z};
    auto t_max_corn = t * vec3{max_corner.x, max_corner.y, max_corner.z};
    
    return {
        t_position,
        t_rotation,
        t_mass_center,
        point{t_min_corn[0], t_min_corn[1], t_min_corn[2]},
        point{t_max_corn[0], t_max_corn[1], t_max_corn[2]}
    };
}

auto detect_properties(const std::shared_ptr<point_cloud>& cloud) -> properties
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    
    pcl::MomentOfInertiaEstimation <point> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute();
    
//    std::vector <float> moment_of_inertia;
//    std::vector <float> eccentricity;
    point min_point_AABB;
    point max_point_AABB;
    point min_point_OBB;
    point max_point_OBB;
    point position_OBB;
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
    rot_mat rot (rotational_matrix_OBB);
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "detect_properties time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    
    return properties(position, rot, mass_center, min_point_AABB, max_point_AABB);
}

