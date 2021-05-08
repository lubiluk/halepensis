#include "object.hpp"
#include "utils.hpp"
#include "surface_recognition.hpp"

#include <algorithm>


Object::Object(std::initializer_list<std::string> const &pcd_files)
{
    for (auto &f : pcd_files) {
        PointCloud::Ptr cloud (new PointCloud);
        cloud = loadCloud(f);
        cloud = removeBackground(cloud);
        cloud = downsample(cloud);
        feature_clouds.push_back(FeatureCloud(cloud));
    }
}

std::vector<PointCloud::Ptr> Object::getPointClouds() const
{
    std::vector<PointCloud::Ptr> out(feature_clouds.size());
    std::transform(feature_clouds.begin(), feature_clouds.end(), out.begin(), [](auto const &f) {
        return f.getPointCloud();
    });
    
    return out;
}

auto Object::findParts() -> void
{
    const SurfaceRecognition surface_recognition;
    parts = surface_recognition.recognize(feature_clouds[0]);
}
