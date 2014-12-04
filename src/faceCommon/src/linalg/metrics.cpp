#include "faceCommon/linalg/metrics.h"

#include <opencv/cv.h>

#include "faceCommon/linalg/common.h"
#include "faceCommon/linalg/matrixconverter.h"

using namespace Face::LinAlg;

Metrics::Ptr MetricsFactory::create(const std::string &name)
{
    if      (name == EuclideanMetric::name())    return new EuclideanMetric();
    else if (name == CosineMetric::name())       return new CosineMetric();
    else if (name == CorrelationMetric::name())  return new CorrelationMetric();
    else if (name == CityblockMetric::name())    return new CityblockMetric();
    else if (name == IntersectionMetric::name()) return new IntersectionMetric();
    else if (name == ChiSquareMetric::name())    return new ChiSquareMetric();

    throw FACELIB_EXCEPTION("unknown metrics: " + name);
}

MahalanobisMetric::MahalanobisMetric(std::vector<Vector> &samples)
{
    learn(samples);
}

void MahalanobisMetric::learn(std::vector<Vector> &samples)
{
    cv::Mat data = Face::LinAlg::MatrixConverter::columnVectorsToDataMatrix(samples);
    cv::Mat covar;
    cv::calcCovarMatrix(data, covar, mean, CV_COVAR_NORMAL | CV_COVAR_COLS, CV_64F);
    cv::invert(covar, invCov, cv::DECOMP_SVD);
}

MahalanobisWeightedMetric::MahalanobisWeightedMetric(std::vector<Vector> &samples)
{
    cv::Mat data = Face::LinAlg::MatrixConverter::columnVectorsToDataMatrix(samples);
    cv::Mat covar;
    cv::calcCovarMatrix(data, covar, mean, CV_COVAR_NORMAL | CV_COVAR_COLS, CV_64F);
    cv::invert(covar, invCov, cv::DECOMP_SVD);
}

double HammingMetric::distance(const Vector &v1, const Vector &v2) const
{
    int n = v1.rows;

    double sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        if (v1(i) != v2(i))
        {
            sum += 1;
        }
    }

    return sum/n;
}

double IntersectionMetric::distance(const Vector &v1, const Vector &v2) const
{
    int n = v1.rows;
    double sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        sum += v1(i) < v2(i) ? v1(i) : v2(i);
    }
    return sum/n;
}

double ChiSquareMetric::distance(const Vector &v1, const Vector &v2) const
{
    int n = v1.rows;
    double sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        sum += pow(v1(i) - v2(i), 2) / (v1(i) + v2(i));
    }
    return sum/n;
}
