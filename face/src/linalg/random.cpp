#include "random.h"

#include <opencv/cv.h>
#include <cassert>

QVector<Matrix> Random::gauss(QVector<double> mean, QVector<double> sigma, int count)
{;
    int m = mean.count();
    assert(m > 0);
    assert(m == sigma.count());

    cv::RNG rng;
    QVector<Matrix> result;
    for (int i = 0; i < count; i++)
    {
        Matrix mat(m, 1, CV_64F);
        for (int j = 0; j < m; j++)
        {
            double rndVal = rng.gaussian(sigma[j]);
            mat(j) = rndVal + mean[j];
        }
        result.append(mat);
    }
    return result;
}
