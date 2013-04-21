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

QVector<Map> Gabor::getRealResponse(Map &map)
{
    QVector<Map> result;
    int n = realKernels.count();
    for (int i = 0; i < n; i++)
    {
        Map re = map;
        re.applyFilter(realKernels[i]);
        result << re;
    }
    return result;
}

QVector<Matrix> Gabor::getRealResponse(Matrix &mat)
{
    QVector<Matrix> result;
    int n = realKernels.count();
    for (int i = 0; i < n; i++)
    {
        Matrix re;
        cv::filter2D(mat, re, CV_64F, realKernels[i]);
        result << re;
    }

    return result;
}

QVector<Map> Gabor::getImagResponse(Map &map)
{
    QVector<Map> result;
    int n = imagKernels.count();
    for (int i = 0; i < n; i++)
    {
        Map im = map;
        im.applyFilter(imagKernels[i]);
        result << im;
    }
    return result;
}

QVector<Matrix> Gabor::getImagResponse(Matrix &mat)
{
    QVector<Matrix> result;
    int n = imagKernels.count();
    for (int i = 0; i < n; i++)
    {
        Matrix im;
        cv::filter2D(mat, im, CV_64F, imagKernels[i]);
        result << im;
    }

    return result;
}

QVector<Map> Gabor::getAbsResponse(Map &map)
{
    QVector<Map> result;
    int n = realKernels.count();
    for (int i = 0; i < n; i++)
    {
        Map re = map;
        Map im = map;
        re.applyFilter(realKernels[i]);
        im.applyFilter(imagKernels[i]);

        Map a(map.w, map.h);
        int m = a.w*a.h;
        for (int j = 0; j < m; j++)
        {
            if (!re.flags[i] || !im.flags[i]) continue;

            a.flags[i] = true;
            a.values[i] = sqrt(re.values[i]*re.values[i] + im.values[i]*im.values[i]);
        }
        result << a;
    }
    return result;
}

QVector<Matrix> Gabor::getAbsResponse(Matrix &mat)
{
    QVector<Matrix> result;
    int n = realKernels.count();
    for (int i = 0; i < n; i++)
    {
        Matrix re;
        cv::filter2D(mat, re, CV_64F, realKernels[i]);

        Matrix im;
        cv::filter2D(mat, im, CV_64F, imagKernels[i]);

        Matrix re2;
        cv::multiply(re, re, re2);
        Matrix im2;
        cv::multiply(im, im, im2);
        Matrix ab2 = re2 + im2;
        Matrix ab;
        cv::sqrt(ab2, ab);
        result << ab;
    }

    return result;
}
