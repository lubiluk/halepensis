#include "mass_center.hpp"


MassCenter::MassCenter(std::string id, Vector position):
SceneEntity(id, position)
{
    
}

auto MassCenter::clone() -> std::shared_ptr<SceneEntity> const
{
    return std::make_shared<MassCenter>(*this);
}
