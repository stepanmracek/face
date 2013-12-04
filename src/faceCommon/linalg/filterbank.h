#ifndef FILTERBANK_H
#define FILTERBANK_H

#include <QVector>

#include "linalg/common.h"
#include "facelib/map.h"

class FilterBank
{
public:
    QVector<Matrix> realKernels;
    QVector<Matrix> imagKernels;

    static Matrix absResponse(const Matrix &image, const Matrix &realKernel, const Matrix &imagKernel);
};

#endif // FILTERBANK_H
