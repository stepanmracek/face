#ifndef GABOR_H
#define GABOR_H

#include "common.h"
#include "kernelgenerator.h"
#include "facelib/map.h"
#include "linalg/filterbank.h"

class Gabor : public FilterBank
{
public:
    Gabor(int size);

    static void createWavelet(Matrix &real, Matrix &imag, int frequency, int orientation);
private:
    static void gaborFunc(int x, int y, double omega, double theta, double sigma, double &real, double &imag);
};

#endif // GABOR_H
