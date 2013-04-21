#ifndef KERNELGENERATOR_H
#define KERNELGENERATOR_H

#include "common.h"

class KernelGenerator
{
public:
    static Matrix gaussianKernel(int size);

    static QVector<Matrix> gaborBank(int size);
};

#endif // KERNELGENERATOR_H
