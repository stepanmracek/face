#include "gausslaguerre.h"

GaussLaguerre::GaussLaguerre(int size)
{
}


unsigned int factorial(unsigned int n)
{
    unsigned int ret = 1;
    for(unsigned int i = 1; i <= n; ++i)
        ret *= i;
    return ret;
}

double over(double n, double k)
{
    return factorial(n)/(factorial(k) * factorial(n-k));
}

double GaussLaguerre::L(double r, int n, int k)
{
    double sum = 0;
    for (int h = 0; h <= K; h++)
    {
        sum += pow(-1, K) * over(n + k, k - h) * pow(r, h) / factorial(h);
    }
}
