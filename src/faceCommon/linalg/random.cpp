#include "random.h"

#include <opencv/cv.h>
#include <cassert>

QVector<Vector> Random::gauss(QVector<double> mean, QVector<double> sigma, int count)
{;
    int m = mean.count();
    assert(m > 0);
    assert(m == sigma.count());

    cv::RNG rng;
    QVector<Vector> result;
    for (int i = 0; i < count; i++)
    {
        Vector vec(m);
        for (int j = 0; j < m; j++)
        {
            double rndVal = rng.gaussian(sigma[j]);
            vec(j) = rndVal + mean[j];
        }
        result.append(vec);
    }
    return result;
}
