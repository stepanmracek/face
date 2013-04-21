#ifndef GABOR_H
#define GABOR_H

#include "common.h"
#include "kernelgenerator.h"
#include "facelib/map.h"

class Gabor
{
public:
    Gabor(int size);

    QVector<Matrix> realKernels;
    QVector<Matrix> imagKernels;

    QVector<Map> getRealResponse(Map &map);
    QVector<Matrix> getRealResponse(Matrix &mat);

    QVector<Map> getImagResponse(Map &map);
    QVector<Matrix> getImagResponse(Matrix &mat);

    QVector<Map> getAbsResponse(Map &map);
    QVector<Matrix> getAbsResponse(Matrix &mat);

private:
    void gaborFunc(int x, int y, double omega, double theta, double sigma, double &real, double &imag);
};

#endif // GABOR_H
