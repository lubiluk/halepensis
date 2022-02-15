#include "scene_visualization.hpp"
#include "geometry.hpp"
#include "task_understanding.hpp"
#include "scene_understanding.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/visualization/pcl_visualizer.h>
#pragma clang diagnostic pop

//std::vector<std::array<int, 3>> color_pallete {
//    {255, 89, 94},
//    {255, 202, 58},
//    {138, 201, 38},
//    {25, 130, 196},
//    {106, 76, 147},
//};

using Color = std::array<double, 3>;

const Color object_color{255, 89, 94};
const Color feature_color{255, 202, 58};

auto vector_to_point(const vec3& vec) -> pcl::PointXYZ
{
    return pcl::PointXYZ{vec.x(), vec.y(), vec.z()};
}

auto translated(const pcl::PointXYZ& point, float x_offset) -> pcl::PointXYZ
{
    auto out = point;
    out.x += x_offset;
    return out;
}

auto view_entity(const scene_entity& entity,
                 const pcl::visualization::PointCloudColorHandlerCustom<point>& color,
                 pcl::visualization::PCLVisualizer& viz,
                 int viewport, const std::string& prefix = "") -> void
{
    using namespace pcl::visualization;
    
    auto vid = std::to_string(viewport);
    
    if (entity.cloud) {
        auto cid = vid + "_cloud_" + prefix + entity.id;
        viz.addPointCloud<point>(entity.cloud, color, cid, viewport);
        viz.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 3, cid, viewport);
        
//        if (entity.type != Entity::Type::object) {
//            auto bid = vid + "_box_" + prefix + entity.id;
//            viz.addCube(entity.min_corner.x, entity.max_corner.x,
//                        entity.min_corner.y, entity.max_corner.y,
//                        entity.min_corner.z, entity.max_corner.z,
//                        1.0, 1.0, 1.0, bid, viewport);
//        }
    }
    
    auto pos = vector_to_point(entity.position);
    auto txt_pos = translated(pos, 0.01);
    auto sid = vid + "_sphere_" + prefix + entity.id;
    auto tid = vid + "_text_" + prefix + entity.id;
    viz.addSphere(pos, 0.005, 1.0, 1.0, 1.0, sid, viewport);
    viz.addText3D(entity.id, txt_pos, 0.01, 1.0, 1.0, 1.0, tid, viewport);
}

auto view_scene(const scene_understanding& scene,
                pcl::visualization::PCLVisualizer& viz,
                int viewport) -> void
{
    using namespace pcl::visualization;
    
    auto vid = std::to_string(viewport);
    auto sid = vid + "_scene_cloud";
    
    viz.addPointCloud<point>(scene.cloud, sid, viewport);
    viz.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, sid, viewport);
    
    for (auto& obj : objects(scene.graph)) {
        PointCloudColorHandlerCustom<point> color{obj.cloud,
            object_color[0], object_color[1], object_color[2]};
        view_entity(obj, color, viz, viewport);
        std::string prefix = obj.id + "_";
        
        for (auto feat : features(obj.id, scene.graph)) {
            PointCloudColorHandlerCustom<point> color{obj.cloud,
                feature_color[0], feature_color[1], feature_color[2]};
            view_entity(scene.graph[feat], color, viz, viewport, prefix);
        }
    }
}

auto view_scenes(const task_understanding& task) -> void
{
    using namespace pcl::visualization;
    
    PCLVisualizer viz("3D Viewer");
    viz.setBackgroundColor(0, 0, 0);
    viz.initCameraParameters();
    viz.setCameraPosition(0, 0, 0, 0, 0, -1, 0, 1, 0);
    
    int v1{0};
    viz.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    int v2{0};
    viz.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    
    view_scene(task.before_scene, viz, v1);
    view_scene(task.after_scene, viz, v2);
    
    viz.spin();
    viz.close();
}
