#pragma once

#include "geometry.hpp"
#include <memory>
#include <vector>
#include <string>

class Entity {
public:
    const std::shared_ptr<PointCloud> cloud;
    const std::string description;
    
    virtual ~Entity() = 0;
    
protected:
    Entity(std::string description) : description(description) { }
};

class Feature: public Entity {
protected:
    Feature(std::string description) : Entity(description) { }
};

class Hole: public Feature {
public:
    Hole(): Feature("Hole") {};
};


class Object: public Entity {
public:
    std::vector<Feature> features;
    
    Object(std::string description) : Entity(description) { }
};

class Relationship {
public:
    Entity& entity1;
    Entity& entity2;
};

class SceneGraph {
public:
    std::vector<std::reference_wrapper<Object>> objects;
    std::vector<Relationship> relationships;
};
