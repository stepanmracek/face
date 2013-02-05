#ifndef ACTIVESHAPEMODEL_H
#define ACTIVESHAPEMODEL_H

#include <QVector>

#include <opencv/cv.h>

#include "linalg/pca.h"
#include "linalg/procrustes.h"
#include "linalg/vector.h"
#include "linalg/metrics.h"
#include "activeshapemodelpoints.h"
#include "linalg/common.h"

class ActiveShapeModelSettings
{
public:
    ActiveShapeModelSettings()
    {
        neighborhoodSize = 5;
        enhancedSearchSize = 10;
    }

    int neighborhoodSize;
    int enhancedSearchSize;
    ActiveShapeModelPointDefinition points;
};

class ActiveShapeModel
{
public:
    PCA pca;
    QVector<MahalanobisMetric> pointMetrics;
    ActiveShapeModelSettings settings;

    ActiveShapeModel(QVector<Matrix> &shapes, QVector<Matrix> &images, ActiveShapeModelSettings settings);

    double b(int eigenValueIndex)
    {
        double eigenVal = fabs(pca.cvPca.eigenvalues.at<double>(eigenValueIndex, 0));
        return 3.0 * sqrt(eigenVal);
    }

    Matrix FitToShape(Matrix &shape, TranslationCoefs &trans, RotateAndScaleCoefs &rotScale, double eps);

    Matrix FitToImage(Matrix &initShape, Matrix &image, TranslationCoefs &trans, RotateAndScaleCoefs &rotScale, double eps);

    void drawModelInstance(Matrix &image, Matrix &shape, double intensity);

private:
    void getNormal(Matrix &shape, int pointIndex, int neighborhoodSize, cv::Point *result);
    void applyConstraintsOnB(Matrix &bvec);
};

#endif // ACTIVESHAPEMODEL_H
