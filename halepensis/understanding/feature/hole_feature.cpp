#include "hole_feature.hpp"

HoleFeature::HoleFeature(std::string id,
                         std::shared_ptr<PointCloud> cloud,
                         Vector position,
                         Quaternion orientation):
CloudEntity(id, cloud, position, orientation)
{
    
}

HoleFeature::HoleFeature(const HoleFeature& original):
CloudEntity(original)
{
    
}

auto HoleFeature::clone() -> std::shared_ptr<SceneEntity> const
{
    return std::make_shared<HoleFeature>(*this);
}
