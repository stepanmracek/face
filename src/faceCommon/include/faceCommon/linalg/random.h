#ifndef RANDOM_H
#define RANDOM_H

#include "common.h"
#include "vector.h"

namespace Face {
namespace LinAlg {

class Random
{
public:
    static std::vector<Vector> gauss(std::vector<double> mean, std::vector<double> sigma, unsigned int count);
};

}
}

#endif // RANDOM_H
