#include "id_provider.hpp"

auto IdProvider::instance() -> IdProvider&
{
    static IdProvider instance;
    return instance;
}

auto IdProvider::next_id(const std::string& key) -> std::string
{
    counters[key] += 1;
    return key + std::to_string(counters[key]);
}
