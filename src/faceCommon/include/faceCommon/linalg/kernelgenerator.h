#ifndef KERNELGENERATOR_H
#define KERNELGENERATOR_H

#include "common.h"

namespace Face {
namespace LinAlg {

class KernelGenerator
{
public:
    static Matrix gaussianKernel(int size);
};

}
}

#endif // KERNELGENERATOR_H
