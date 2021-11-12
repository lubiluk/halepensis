#include "relation.hpp"

using namespace std;

Relation::Relation(Type type):
type(type)
{
    
}

Relation::Relation():
type(Type::has)
{
    
}

auto Relation::description() const -> string
{
    switch (type) {
        case Type::has:
            return "";
        case Type::above:
            return "above";
        case Type::below:
            return "below";
        case Type::apart:
            return "apart";
        case Type::touching:
            return "touching";
        default:
            return "";
    }
}
