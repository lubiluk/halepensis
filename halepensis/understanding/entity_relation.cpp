#include "entity_relation.hpp"

using namespace std;
using boost::optional;
using boost::none;

auto relation_type_to_string(relation_type type) -> string
{
    switch (type) {
        case relation_type::has:
            return "";
        case relation_type::above:
            return "above";
        case relation_type::below:
            return "below";
        case relation_type::apart:
            return "apart";
        case relation_type::touching:
            return "touching";
        case relation_type::inside:
            return "inside";
        default:
            return "unknown_relation";
    }
}

auto relation_type_from_string(const string& string) -> optional<relation_type>
{
    if (string == relation_type_to_string(relation_type::has)) {
        return relation_type::has;
    }
    
    if (string == relation_type_to_string(relation_type::above)) {
        return relation_type::above;
    }
    
    if (string == relation_type_to_string(relation_type::below)) {
        return relation_type::below;
    }
    
    if (string == relation_type_to_string(relation_type::apart)) {
        return relation_type::apart;
    }
    
    if (string == relation_type_to_string(relation_type::touching)) {
        return relation_type::touching;
    }
    
    if (string == relation_type_to_string(relation_type::inside)) {
        return relation_type::inside;
    }
    
    return none;
}

entity_relation::entity_relation(relation_type type):
type(type)
{
    
}

entity_relation::entity_relation():
type(relation_type::has)
{
    
}

entity_relation::entity_relation(const string& description):
type(*relation_type_from_string(description))
{
    
}

auto entity_relation::description() const -> string
{
    return relation_type_to_string(type);
}
