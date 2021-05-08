#include "surface_recognition.hpp"

SurfaceRecognition::SurfaceRecognition(): SacPartRecognition(pcl::SACMODEL_PLANE, PartType::surface)
{
    
}
