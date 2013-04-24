#ifndef FILTERBANK_H
#define FILTERBANK_H

#include <QVector>

#include "common.h"
#include "facelib/map.h"

class FilterBank
{
public:
    QVector<Matrix> realKernels;
    QVector<Matrix> imagKernels;

    QVector<Map> getRealResponse(Map &map);
    QVector<Matrix> getRealResponse(Matrix &mat);

    QVector<Map> getImagResponse(Map &map);
    QVector<Matrix> getImagResponse(Matrix &mat);

    QVector<Map> getAbsResponse(Map &map);
    QVector<Matrix> getAbsResponse(Matrix &mat);
};

#endif // FILTERBANK_H
