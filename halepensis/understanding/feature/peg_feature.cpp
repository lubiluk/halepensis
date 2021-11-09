#include "peg_feature.hpp"

PegFeature::PegFeature(std::string id,
                         std::shared_ptr<PointCloud> cloud,
                         Vector position,
                         Quaternion orientation):
CloudEntity(id, cloud, position, orientation)
{
    
}

PegFeature::PegFeature(const PegFeature& original):
CloudEntity(original)
{
    
}

auto PegFeature::clone() -> std::shared_ptr<SceneEntity> const
{
    return std::make_shared<PegFeature>(*this);
}
