#include "gabor.h"

Gabor::Gabor(int size)
{
    assert(size % 2 == 1);

    // frequency
    for (int frequency = 1; frequency <= 5; frequency++)
    {
        // orientation
        for (int orientation = 1; orientation <= 8; orientation++)
        {
            Matrix real(size, size);
            Matrix imag(size, size);

            createWavelet(real, imag, frequency, orientation);

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

void Gabor::createWavelet(Matrix &real, Matrix &imag, int frequency, int orientation)
{
    double omegaMax = M_PI_2;
    double lambda = M_SQRT2;
    double omega = omegaMax * pow(lambda, -(frequency - 1));
    double sigma = M_PI / omega;

    int size = pow(2, ceil(log2(sigma*6)));
    real = Matrix(size, size);
    imag = Matrix(size, size);

    double theta = (orientation-1)*M_PI*0.125;

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
}
