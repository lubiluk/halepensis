#pragma once

#include <string>

class Error {
    public:
    std::string message;

    Error(std::string message): message(message) {}
};


class error {
    public:
    std::string message;

    error(std::string message): message(message) {}
};
