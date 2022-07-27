#include "base.hpp"
#include "filtering.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/common/io.h>
#include <pcl/features/organized_edge_detection.h>
#pragma clang diagnostic pop

using pcl::OrganizedEdgeFromNormals;
using pcl::PointCloud;
using std::vector;
using pcl::Label;
using std::make_shared;
using std::shared_ptr;

auto detect_base(const std::shared_ptr<point_cloud> &cloud)
-> vector<shared_ptr<point_cloud>>
{
    auto indics = make_shared<point_indices>();
    auto lowest_indx = 0;
    
    for (int i = 0; i < cloud->size(); ++i) {
        if ((*cloud)[i].y < (*cloud)[lowest_indx].y) {
            lowest_indx = i;
        }
    }
    
    for (int i = 0; i < cloud->size(); ++i) {
        if (abs((*cloud)[i].y - (*cloud)[lowest_indx].y) < 0.0075) {
            indics->indices.push_back(i);
        }
    }
    
    auto base = extract_cloud(cloud, indics);
    
    return { base };
}
