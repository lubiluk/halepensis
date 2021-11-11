#include "mass_center_feature.hpp"


MassCenterFeature::MassCenterFeature(std::string id, Vector position):
SceneEntity(id, position)
{
    
}

auto MassCenterFeature::clone() -> std::shared_ptr<SceneEntity> const
{
    return std::make_shared<MassCenterFeature>(*this);
}
