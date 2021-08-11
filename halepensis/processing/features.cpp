#include "features.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/features/boundary.h>
#include <pcl/search/kdtree.h>
#pragma clang diagnostic pop


auto estimate_boundaries(const std::shared_ptr<point_cloud>& cloud, const std::shared_ptr<surface_normals>& normals)
-> std::shared_ptr<point_indices>
{
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<point, normal, pcl::Boundary> est;
    est.setInputCloud(cloud);
    est.setInputNormals(normals);
//    est.setRadiusSearch(0.002);   // 2cm radius
    est.setAngleThreshold(M_PI_2);
    est.setKSearch(20);
    est.setSearchMethod(typename pcl::search::KdTree<point>::Ptr (new pcl::search::KdTree<point>));
    est.compute(boundaries);
    
    auto inliers = std::make_shared<point_indices>();
    
    for (int i = 0; i < boundaries.size(); ++i)
    {
        if (boundaries.points[i].boundary_point == 1)
        {
            inliers->indices.push_back(i);
        }
    }
    
    return inliers;
}
