#pragma once

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
