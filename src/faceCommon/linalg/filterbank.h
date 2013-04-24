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

    virtual QVector<Map> getRealResponse(Map &map) = 0;
    virtual QVector<Matrix> getRealResponse(Matrix &mat) = 0;

    virtual QVector<Map> getImagResponse(Map &map) = 0;
    virtual QVector<Matrix> getImagResponse(Matrix &mat) = 0;

    virtual QVector<Map> getAbsResponse(Map &map) = 0;
    virtual QVector<Matrix> getAbsResponse(Matrix &mat) = 0;
};

#endif // FILTERBANK_H
