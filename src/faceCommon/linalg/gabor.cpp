#include "gabor.h"

Gabor::Gabor(int size)
{
    assert(size % 2 == 1);

    // frequency
    double omegaMax = M_PI_2;
    double lambda = M_SQRT2;
    for (int m = 1; m <= 5; m++)
    {
        double omega = omegaMax * pow(lambda, -(m - 1));
        double sigma = M_PI / omega;

        // orientation
        for (int n = 1; n <= 8; n++)
        {
            double theta = (n-1)*M_PI*0.125;

            Matrix real(size, size);
            Matrix imag(size, size);

            for (int y = 0; y < size; y++)
            {
                for (int x = 0; x < size; x++)
                {
                    double r,i;
                    gaborFunc(x-size/2, y-size/2, omega, theta, sigma, r, i);
                    real(y, x) = r;
                    imag(y, x) = i;
                }
            }

            realKernels << real;
            imagKernels << imag;
        }
    }
}

void Gabor::gaborFunc(int x, int y, double omega, double theta, double sigma, double &real, double &imag)
{
    double xprime = x*cos(theta) + y*sin(theta);
    double yprime = -x*sin(theta) + y*cos(theta);
    double sigma2 = sigma*sigma;
    double a = 1.0 / (2.0 * M_PI * sigma2);
    double b = exp(- (xprime*xprime + yprime*yprime)/(2*sigma2));
    double cReal = cos(omega*xprime);
    double cImag = sin(omega*xprime);
    double d = exp(- 0.5*omega*omega*sigma2);

    real = a*b*(cReal - d);
    imag = a*b*(cImag - d);
}

