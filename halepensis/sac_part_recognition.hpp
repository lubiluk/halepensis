#pragma once

#include "types.hpp"
#include "part_recognition.hpp"

class SacPartRecognition: public PartRecognition {
public:
    const int model_type;
    const PartType part_type;
    SacPartRecognition(const int model_type, const PartType part_type);
    ~SacPartRecognition() = 0;
    auto recognize(const FeatureCloud& feature_cloud) const -> std::vector<const Part> override;
};
