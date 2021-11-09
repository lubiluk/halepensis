#include "scene_object.hpp"


SceneObject::SceneObject(std::string id,
                         std::shared_ptr<PointCloud> cloud,
                         Vector position,
                         Quaternion orientation):
CloudEntity(id, cloud, position, orientation)
{
    
}

SceneObject::SceneObject(const SceneObject& original):
CloudEntity(original)
{
    
}

auto SceneObject::clone() -> std::shared_ptr<SceneEntity> const
{
    return std::make_shared<SceneObject>(*this);
}
