#pragma once
#include <stdexcept>

namespace crowhttp
{
    struct bad_request : public std::runtime_error
    {
        bad_request(const std::string& what_arg)
            : std::runtime_error(what_arg) {}

        bad_request(const char* what_arg)
            : std::runtime_error(what_arg) {}
    };
}