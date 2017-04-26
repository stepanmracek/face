#pragma once

#include <map>

class EvaluateRealsense
{
    static std::map<int, std::string> getNames();

public:
    static void identification();
    static void verification();
};
