#pragma once

#include <string>

class Relation {
public:
    enum class Type {
        has,
        below,
        above,
        apart,
        touching,
        inside
    };
    
    Type type;
    
    Relation(Type type);
    Relation();
    Relation(const Relation& original) = default;
    
    auto description() const -> std::string;
};

