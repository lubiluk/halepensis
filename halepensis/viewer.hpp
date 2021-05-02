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
    std::vector<std::array<int, 3>> color_pallete = {
        {255, 89, 94},
        {255, 202, 58},
        {138, 201, 38},
        {25, 130, 196},
        {106, 76, 147},
    };

    static void view(const Object &object);
    static void view(const Scene &scene);
    static void view(const PointCloud::Ptr cloud);
    static void view(const std::vector<const PointCloud::Ptr> &clouds);
    static void view(std::initializer_list<PointCloud::Ptr> clouds);
};
