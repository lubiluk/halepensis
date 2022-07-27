#include "edge.hpp"

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

auto detect_edges(const std::shared_ptr<point_cloud> &cloud)
-> vector<shared_ptr<point_cloud>>
{
    const auto normals = std::make_shared<surface_normals>();
    pcl::copyPointCloud(*cloud, *normals);
    
    OrganizedEdgeFromNormals<point, normal, Label> oed;
    oed.setInputNormals (normals);
    oed.setInputCloud (cloud);
    oed.setDepthDisconThreshold (0.01);
    oed.setMaxSearchNeighbors (0.4);
    oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
    PointCloud<Label> labels;
    vector<point_indices> label_indices;
    oed.compute (labels, label_indices);
    
    vector<shared_ptr<point_cloud>> edges;
    
    for (auto& i : label_indices) {
        if (i.indices.size() == 0) {
            continue;
        }
        
        auto c = make_shared<point_cloud>();
        copyPointCloud (*cloud, i, *c);
        edges.push_back(c);
    }
    
    return edges;
}
