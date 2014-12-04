#ifndef FILTERBANK_H
#define FILTERBANK_H

#include "common.h"

namespace Face {
namespace LinAlg {

class FilterBank
{
public:
    std::vector<Matrix> realKernels;
    std::vector<Matrix> imagKernels;

    static Matrix absResponse(const Matrix &image, const Matrix &realKernel, const Matrix &imagKernel);
};

}
}

#endif // FILTERBANK_H
