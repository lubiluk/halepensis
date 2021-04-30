#pragma once

#include "types.hpp"
#include "object.hpp"
#include "scene.hpp"

#include <pcl/visualization/pcl_visualizer.h>

class Viewer
{
public:
    static void view(const Object &object);
    static void view(const Scene &scene);
    static void view(const PointCloud::Ptr cloud);
};