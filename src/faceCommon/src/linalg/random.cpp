#include "faceCommon/linalg/random.h"

#include <opencv/cv.h>

using namespace Face::LinAlg;

std::vector<Vector> Random::gauss(std::vector<double> mean, std::vector<double> sigma, unsigned int count)
{
    unsigned int m = mean.size();
    if (m == 0 || m != sigma.size()) throw FACELIB_EXCEPTION("invalid input parameters");

    cv::RNG rng;
    std::vector<Vector> result;
    for (unsigned int i = 0; i < count; i++)
    {
        Vector vec(m);
        for (unsigned int j = 0; j < m; j++)
        {
            double rndVal = rng.gaussian(sigma[j]);
            vec(j) = rndVal + mean[j];
        }
        result.push_back(vec);
    }
    return result;
}
