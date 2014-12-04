#ifndef DATAVIZ_H
#define DATAVIZ_H

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

#endif // DATAVIZ_H
