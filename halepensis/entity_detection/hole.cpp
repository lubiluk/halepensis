#include "hole.hpp"
#include "features.hpp"
#include "filtering.hpp"
#include "regions.hpp"
#include "search.hpp"
#include "normals.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree_flann.h>
#pragma clang diagnostic pop

#include <chrono>

auto detect_holes(const std::shared_ptr<point_cloud> &cloud, float pixel_radius)
-> std::vector<std::shared_ptr<point_cloud>>
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    
    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = -1;
    coefficients->values[3] = 0;
    
    auto cloud_projected = std::make_shared<point_cloud>();
    // Create the filtering object
    pcl::ProjectInliers<point> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    compute_normals(cloud_projected);
    
    float minX, maxX, minY, maxY = 0.0f;
    
    minX = maxX = (*cloud_projected)[0].x;
    minY = maxY = (*cloud_projected)[0].y;
    
    for (auto& point : *cloud_projected) {
        if (point.x < minX) minX = point.x;
        if (point.x > maxX) maxX = point.x;
        if (point.y < minY) minY = point.y;
        if (point.y > maxY) maxY = point.y;
    }
    
    minX = floor(minX * 100) / 100;
    maxX = ceil(maxX * 100) / 100;
    minY = floor(minY * 100) / 100;
    maxY = ceil(maxY * 100) / 100;
    
    int cols = abs(maxX - minX) / pixel_radius;
    int rows = abs(maxY - minY) / pixel_radius;
    
    pcl::KdTreeFLANN<point> kdtree;
    kdtree.setInputCloud(cloud_projected);
    
    std::vector<std::vector<bool>> grid;
    
    for (int r = 0; r < rows; ++r) {
        grid.emplace_back();
        
        for (int c = 0; c < cols; ++c) {
            point search_point {minX + c * pixel_radius, minY + r * pixel_radius, 0};
            std::vector<int> point_idx_radius_search;
            std::vector<float> point_radius_squared_distance;
            kdtree.radiusSearch(search_point, pixel_radius, point_idx_radius_search, point_radius_squared_distance);
            
            grid[r].emplace_back(point_idx_radius_search.size() > 0);
        }
    }
    
    std::vector<std::pair<int, int>> hole_inds;
    
    for (int r = 0; r < rows; ++r) {
        grid.emplace_back();
        for (int c = 0; c < cols; ++c) {
            if (grid[r][c]) continue;
            
            bool before_x, before_y, after_x, after_y;
            before_x = before_y = after_x = after_y = false;
            
            for (int i = 0; i < c; ++i) {
                if (grid[r][i]) {
                    before_x = true;
                    break;
                }
            }
            
            for (int i = c + 1; i < cols; ++i) {
                if (grid[r][i]) {
                    after_x = true;
                    break;
                }
            }
            
            for (int i = 0; i < r; ++i) {
                if (grid[i][c]) {
                    before_y = true;
                    break;
                }
            }
            
            for (int i = r + 1; i < rows; ++i) {
                if (grid[i][c]) {
                    after_y = true;
                    break;
                }
            }
            
            if (before_x && before_y && after_x && after_y) {
                hole_inds.emplace_back(r, c);
            }
        }
    }
    
    std::vector<std::vector<std::pair<int, int>>> clusters;
    std::vector<std::pair<int, int>> clustered;
    
    for (auto pair : hole_inds) {
        int r, c;
        std::tie(r, c) = pair;
        
        if (std::find(clustered.begin(), clustered.end(), pair) != clustered.end()) {
            continue;
        }
        
        clustered.push_back(pair);
        clusters.push_back({pair});
        auto& cluster = clusters.back();
        
        int min_r, max_r, min_c, max_c;
        min_r = max_r = r;
        min_c = max_c = c;
        
        auto found = false;
        
        do {
            found = false;
            
            for (auto other_pair : hole_inds) {
                if (std::find(clustered.begin(), clustered.end(), other_pair) != clustered.end()) {
                    continue;
                }
                
                int other_r, other_c;
                std::tie(other_r, other_c) = other_pair;
                
                if (other_r >= min_r - 1 && other_r <= max_r + 1 && other_c >= min_c - 1 && other_c <= max_c + 1) {
                    found = true;
                    cluster.push_back(other_pair);
                    clustered.push_back(other_pair);
                    
                    min_r = std::min(min_r, other_r);
                    max_r = std::max(max_r, other_r);
                    min_c = std::min(min_c, other_c);
                    max_c = std::max(max_c, other_c);
                }
            }
        } while (found);
    }
    
    
    std::vector<std::vector<int>> index_clusters;
    
    for (auto& cluster : clusters) {
        std::vector<int> index_cluster;
        
        for (auto pair : cluster) {
            int r, c;
            std::tie(r, c) = pair;
            
            point search_point {minX + c * pixel_radius, minY + r * pixel_radius, 0};
            std::vector<int> point_idx_radius_search;
            std::vector<float> point_radius_squared_distance;
            kdtree.radiusSearch(search_point, 2 * pixel_radius, point_idx_radius_search, point_radius_squared_distance);
            index_cluster.insert(index_cluster.end(), point_idx_radius_search.begin(), point_idx_radius_search.end());
        }
        
        index_clusters.push_back(index_cluster);
    }
    
    std::vector<std::shared_ptr<point_cloud>> holes;
    std::transform(index_clusters.begin(), index_clusters.end(), std::back_inserter(holes), [&cloud](const auto& i) -> auto
    {
        pcl::PointIndices::Ptr indices (new pcl::PointIndices);
        indices->indices = i;
        return extract_cloud(cloud, indices);
    });
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "detect_holes time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    
    return holes;
}
