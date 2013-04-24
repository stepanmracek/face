#include "gausslaguerre.h"

GaussLaguerre::GaussLaguerre(int size)
{
    for (int j = 2; j <= 2; j++)
    {
        for (int n = 1; n <= 5; n++)
        {
            for (int k = 0; k <= 4; k++)
            {
                Matrix re = Matrix::zeros(size, size);
                Matrix im = Matrix::zeros(size, size);

                for (int y = 0; y < size; y++)
                {
                    double realY = size/2 - y;
                    for (int x = 0; x < size; x++)
                    {
                        double realX = size/2 - x;
                        double r = sqrt(realX*realX + realY*realY);
                        double theta = atan2(realY, realX);

                        re(y, x) = h(r, theta, n, k, j) * cos(n * theta);
                        im(y, x) = h(r, theta, n, k, j) * sin(n * theta);
                    }
                }

                realKernels << re;
                imagKernels << im;
                Common::printMatrix(re);
                Common::printMatrix(im);
            }
        }
    }
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
    return ((double)factorial(n))/(factorial(k) * factorial(n-k));
}

double GaussLaguerre::L(double r, int n, int k)
{
    double sum = 0;
    for (int h = 0; h <= k; h++)
    {
        sum += pow(-1, k) * over(n + k, k - h) * pow(r, h) / factorial(h);
    }
    return sum;
}

double GaussLaguerre::h(double r, double theta, int n, int k, int j)
{
    return pow(-1, k) * pow(2, (fabs(n)+1.0)/2.0) * pow(M_PI, fabs(n)/2.0) *
            pow((((double)(factorial(k)))/factorial(fabs(n) + k)), 0.5) * pow(r, fabs(n)) *
            L(2 * M_PI * r * r, n, k) * exp(-M_PI * r * r); // * exp(j * n * theta);
}
