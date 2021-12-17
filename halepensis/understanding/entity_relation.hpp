#pragma once

#include <string>
#include <boost/optional.hpp>

enum class relation_type {
    has,
    below,
    above,
    apart,
    touching,
    inside
};

auto relation_type_to_string(relation_type type) -> std::string;
auto relation_type_from_string(const std::string& string) -> boost::optional<relation_type>;

class entity_relation {
public:
    relation_type type;
    
    entity_relation(relation_type type);
    entity_relation();
    entity_relation(const std::string& description);
    entity_relation(const entity_relation& original) = default;
    
    auto description() const -> std::string;
};

