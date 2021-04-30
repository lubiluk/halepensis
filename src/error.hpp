#pragma once

#include <string>

class Error {
    public:
    std::string message;

    Error(std::string message): message(message) {}
};