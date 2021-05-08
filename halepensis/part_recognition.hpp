#pragma once

#include "types.hpp"
#include "feature_cloud.hpp"
#include "part.hpp"

#include <vector>

class PartRecognition {
public:
    virtual ~PartRecognition() {};
    virtual auto recognize(const FeatureCloud& feature_cloud) const -> std::vector<const Part> = 0;
};
