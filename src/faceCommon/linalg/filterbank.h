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

    /*QVector<Matrix> getAbsRealImagResponse(const Matrix &mat,
                                           const QVector<int> *absSelectedIndicies = 0,
                                           const QVector<int> *realSelectedIndicies = 0,
                                           const QVector<int> *imagSelectedIndicies = 0) const;

    QVector<Map> getRealResponse(const Map &map) const;
    QVector<Matrix> getRealResponse(const Matrix &mat,
                                    const QVector<int> *selectedIndicies = 0) const;

    QVector<Map> getImagResponse(const Map &map) const;
    QVector<Matrix> getImagResponse(const Matrix &mat,
                                    const QVector<int> *selectedIndicies = 0) const;

    QVector<Map> getAbsResponse(const Map &map) const;
    QVector<Matrix> getAbsResponse(const Matrix &mat,
                                   const QVector<int> *selectedIndicies = 0) const;*/

    static Matrix absResponse(const Matrix &image, const Matrix &realKernel, const Matrix &imagKernel);
};

#endif // FILTERBANK_H
