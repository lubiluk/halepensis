#pragma once


class Relation {
public:
    enum class Type {
        has,
        below,
        above,
        apart,
        touching
    };
    
    Type type;
    
    Relation(Type type);
    Relation();
    Relation(const Relation& original) = default;
};

