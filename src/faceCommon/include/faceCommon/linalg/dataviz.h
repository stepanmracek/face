#pragma once

#include "common.h"

namespace Face {
namespace LinAlg {

class DataViz
{
public:
    static void ToGnuplotFile(std::vector<Matrix> &vectors, const std::string &path);
};

}
}
