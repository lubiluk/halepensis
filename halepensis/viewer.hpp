#pragma once

#include "types.hpp"
#include "object.hpp"
#include "scene.hpp"

#include <vector>
#include <initializer_list>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/visualization/pcl_visualizer.h>
#pragma clang diagnostic pop

class Viewer
{
public:
    static std::vector<std::array<int, 3>> color_pallete;

    static void view(const PointCloud::ConstPtr cloud);
    static void view(const std::vector<PointCloud::ConstPtr> & clouds, bool split_viewports = false);
    static void view(const std::initializer_list<PointCloud::ConstPtr> &clouds, bool split_viewports = false);
};
