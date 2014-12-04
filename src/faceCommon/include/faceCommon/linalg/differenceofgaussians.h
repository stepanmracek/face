#ifndef DIFFERENCEOFGAUSSIANS_H
#define DIFFERENCEOFGAUSSIANS_H

#include "common.h"

namespace Face {
namespace LinAlg {

class DifferenceOfGaussians
{
public:
    static Matrix dog(const Matrix &input, int kernel1Size, int kernel2Size, bool equalize);
};

}
}

#endif // DIFFERENCEOFGAUSSIANS_H
