#pragma once

#include <string>
#include <map>

class IdProvider {
public:
    static auto instance() -> IdProvider&;
    auto next_id(const std::string& key) -> std::string;
private:
    IdProvider() {}
    IdProvider(IdProvider const&);     // Don't Implement.
    void operator=(const IdProvider&); // Don't implement
    
    std::map<std::string, unsigned> counters;
};
