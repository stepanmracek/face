#include "metrics.h"

#include <opencv/cv.h>

#include "common.h"
#include "matrixconverter.h"

MahalanobisMetric::MahalanobisMetric(QVector<Matrix> &samples)
{
    learn(samples);
}

void MahalanobisMetric::learn(QVector<Matrix> &samples)
{
    cv::Mat data = MatrixConverter::columnVectorsToDataMatrix(samples);
    cv::Mat covar;
    cv::calcCovarMatrix(data, covar, mean, CV_COVAR_NORMAL | CV_COVAR_COLS, CV_64F);
    cv::invert(covar, invCov, cv::DECOMP_SVD);
}

MahalanobisWeightedMetric::MahalanobisWeightedMetric(QVector<Matrix> &samples)
{
    cv::Mat data = MatrixConverter::columnVectorsToDataMatrix(samples);
    cv::Mat covar;
    cv::calcCovarMatrix(data, covar, mean, CV_COVAR_NORMAL | CV_COVAR_COLS, CV_64F);
    cv::invert(covar, invCov, cv::DECOMP_SVD);
}
