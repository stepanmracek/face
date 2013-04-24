#ifndef GAUSSLAGUERRE_H
#define GAUSSLAGUERRE_H

#include "filterbank.h"

class GaussLaguerre : public FilterBank
{
public:
    GaussLaguerre(int size);

private:
    double L(double r, int n, int k);
    double h(double r, double theta, int n, int k, int j);
};

#endif // GAUSSLAGUERRE_H
