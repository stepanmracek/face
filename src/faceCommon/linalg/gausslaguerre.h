#ifndef GAUSSLAGUERRE_H
#define GAUSSLAGUERRE_H

#include "filterbank.h"

class GaussLaguerre : public FilterBank
{
public:
    GaussLaguerre(int size);

    static void createWavelet(Matrix &real, Matrix &imag, int n, int k, int j);

private:
    static double L(double r, int n, int k);
    static double h(double r, double theta, int n, int k, int j);
};

#endif // GAUSSLAGUERRE_H
