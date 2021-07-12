//
//  part.cpp
//  halepensis
//
//  Created by Paweł Gajewski on 06/05/2021.
//

#include "part.hpp"

Part::Part(PartType type, pcl::PointIndices::ConstPtr indices, PointCloud::ConstPtr cloud, pcl::ModelCoefficients::Ptr coefficients):
    type(type), indices(indices), cloud(cloud), coefficients(coefficients)
{
    
}
