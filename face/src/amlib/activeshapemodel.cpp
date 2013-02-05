#include "activeshapemodel.h"

#include <cassert>
#include <limits>
#include <opencv/highgui.h>

#include "linalg/procrustes.h"
#include "lineprocessing.h"

void normalizeMahalanobisSample(Matrix &sample)
{
    int centerIndex = sample.rows/2;
    double centerValue = sample(centerIndex, 0);
    for (int i = 0; i < sample.rows; i++)
    {
        sample(i, 0) = sample(i, 0) - centerValue;
    }
}

void ActiveShapeModel::getNormal(Matrix &shape, int pointIndex, int neighborhoodSize, cv::Point *result)
{
    int n = shape.rows/2;
    cv::Point2d normal = LineProcessing::shapeNormal(pointIndex, shape, settings.points);
    int x = shape(pointIndex, 0);
    int y = shape(pointIndex+n, 0);

    LineProcessing::neighborhood(x, y, normal.x, normal.y, neighborhoodSize, result);
}

ActiveShapeModel::ActiveShapeModel(QVector<Matrix> &shapes, QVector<Matrix> &images, ActiveShapeModelSettings settings)
{
    this->settings = settings;

    qDebug() << "Initializing Active Shape Model";
    int shapesCount = shapes.count();
    assert(shapesCount == images.count());
    assert(shapesCount > 1);
    int n = shapes[0].rows/2;

    QVector<Matrix> samples[n];

    qDebug() << "  ... reading images";

    // shape point data
    for (int curSample = 0; curSample < shapesCount; curSample++)
    {
        Matrix &shape = shapes[curSample];
        Matrix &image = images[curSample];

        for (int curShapePoint = 0; curShapePoint < n; curShapePoint++)
        {
            int lc = (2*settings.neighborhoodSize+1);
            cv::Point line[lc];
            getNormal(shape, curShapePoint, settings.neighborhoodSize, line);

            Matrix sample(lc, 1, CV_64F);
            for (int curNormalPoint = 0; curNormalPoint < lc; curNormalPoint++)
            {
                double value = image(line[curNormalPoint].y, line[curNormalPoint].x);
                sample(curNormalPoint, 0) = value;
            }
            normalizeMahalanobisSample(sample);
            samples[curShapePoint].append(sample);
        }
    }

    // mahalanobis metrics
    qDebug() << "  ... calculating mahalanobis metrics";
    for (int i = 0; i < n; i++)
    {
        /*if (i == 3)
        {
            if (QFile::exists("mahalanabobisSamples"))
                QFile::remove("mahalanabobisSamples");
            int count = samples[i].count();
            for (int j = 0; j < count; j++)
                Vector::toFileWithIndicies(samples[i][j], "mahalanabobisSamples", true);
        }*/
        MahalanobisMetric m(samples[i]);
        pointMetrics.append(m);
    }

    // pca
    qDebug() << "  ... doing PCA";
    Procrustes::procrustesAnalysis(shapes);
    pca.learn(shapes);
}

void createSubSample(Matrix &image, cv::Point *samplePointsCoords, int startIndex, int len, Matrix &result)
{
    for (int i = startIndex; i < startIndex+len; i++)
    {
        double value = image(samplePointsCoords[i].y, samplePointsCoords[i].x);

        //qDebug() << i;
        assert(i-startIndex < result.rows);
        result(i-startIndex, 0) = value;
    }

    normalizeMahalanobisSample(result);
}

Matrix ActiveShapeModel::FitToImage(Matrix &initShape, Matrix &image,
                                     TranslationCoefs &trans, RotateAndScaleCoefs &rotScale,
                                     double eps)
{
    Matrix initialImage = image.clone();
    drawModelInstance(initialImage, initShape, 1);
    cv::imshow("testFitToImage", initialImage);
    cv::waitKey();

    qDebug() << "Fitting model to given image";
    int n = initShape.rows/2;
    assert(n == settings.points.points.count());

    int subSamplesCount = 2*(settings.enhancedSearchSize - settings.neighborhoodSize)+1;
    int subSampleLen = 2*settings.neighborhoodSize + 1;
    int sampleLen = 2*settings.enhancedSearchSize + 1;

    Matrix shape = initShape.clone();
    Matrix b = Matrix::zeros(pca.getModes(), 1);
    Matrix newB = Matrix::zeros(pca.getModes(), 1);
    Matrix delta = Matrix::zeros(pca.getModes(), 1);
    Matrix subSample = Matrix::zeros(subSampleLen, 1);
    cv::Point sampledProfileCoords[sampleLen];
    bool end = false;
    int iteration = 0;

    int counter = 0;
    while (!end)
    {
        // try to estimate better position for each point
        for (int i = 0; i < n; i++)
        {
            getNormal(shape, i, settings.enhancedSearchSize, sampledProfileCoords);
            cv::Point newOptimalPoint;
            double minDistance = 1e300; // std::numeric_limits<double>::max();

            for (int startIndex = 0; startIndex < subSamplesCount; startIndex++)
            {
                createSubSample(image, sampledProfileCoords, startIndex, subSampleLen, subSample);
                double d = pointMetrics[i].distance(subSample);
                if (d < minDistance)
                {
                    minDistance = d;
                    newOptimalPoint = sampledProfileCoords[startIndex+settings.neighborhoodSize];
                }
            }

            shape(i, 0) = newOptimalPoint.x;
            shape(i+n, 0) = newOptimalPoint.y;
        }

        Matrix imageResult = image.clone();
        drawModelInstance(imageResult, shape, 0.5);
        cv::imshow("testFitToImage", imageResult);
        cv::imwrite((QString::number(counter++) + ".png").toStdString(), imageResult*255);
        cv::waitKey(0);

        // fit the model to the new points
        newB = FitToShape(shape, trans, rotScale, eps);
        delta = newB - b;
        double diff = Vector::sqrMagnitude(delta);
        b = newB;

        iteration += 1;
        qDebug() << " => Fit to image iteration:" << iteration << diff;
        if (diff <= eps || iteration > 1000)
            end = true;

        // recalculate shape
        shape = pca.backProject(b);
        Procrustes::rotateAndScale(shape, rotScale);
        Procrustes::translate(shape, trans);

        // debug: draw result
        imageResult = image.clone();
        drawModelInstance(imageResult, shape, 1);
        cv::imshow("testFitToImage", imageResult);
        cv::imwrite((QString::number(counter++) + ".png").toStdString(), imageResult*255);
        cv::waitKey(0);
    }

    return b;
}

void ActiveShapeModel::applyConstraintsOnB(Matrix &bvec)
{
    int n = bvec.rows;
    assert(n == pca.cvPca.eigenvalues.rows);

    for (int i = 0; i < n; i++)
    {
        double val = bvec(i, 0);
        double maxb = b(i);
        if (val > maxb)
            bvec(i, 0) = maxb;
        else if (val < -maxb)
            bvec(i, 0) = -maxb;
    }
}

Matrix ActiveShapeModel::FitToShape(Matrix &shape, TranslationCoefs &trans, RotateAndScaleCoefs &rotScale, double eps)
{
    //qDebug() << "Fitting model to desired shape";
    assert(eps >= 0);
    int n = shape.rows/2;
    assert(n == settings.points.points.count());

    Matrix centralizedDesiredShape = shape.clone();
    TranslationCoefs centTranslationCoefs = Procrustes::centralizedTranslation(centralizedDesiredShape);
    Procrustes::translate(centralizedDesiredShape, centTranslationCoefs);

    Matrix B = Matrix::zeros(pca.getModes(), 1);
    Matrix newB = Matrix::zeros(pca.getModes(), 1);
    Matrix delta = Matrix::zeros(pca.getModes(), 1);
    Matrix desiredShape;
    Matrix modelInstance;
    trans.xt = 0;
    trans.yt = 0;
    rotScale.s = 1;
    rotScale.theta = 0.0;

    bool end = false;
    int iteration = 0;
    RotateAndScaleCoefs c;
    while (!end)
    {
        // instantiate model
        desiredShape = centralizedDesiredShape.clone();
        modelInstance = pca.backProject(B);

        // estimate optimal align of model instance to desired shape
        c = Procrustes::align(modelInstance, desiredShape);
        RotateAndScaleCoefs invC(1/c.s, -c.theta);

        // rotate and scale desired shape back to model (using inverse transform)
        Procrustes::rotateAndScale(desiredShape, invC);

        // Project desired shape into the tangent plane to x by scaling by 1/(y.mean(x)).
        Matrix mean(pca.cvPca.mean);
        double dot = Vector::dot(desiredShape, mean);
        Vector::mul(desiredShape, 1/dot);

        newB = pca.project(desiredShape);
        applyConstraintsOnB(newB);

        // determine whether we have to end
        delta = newB - B;
        double diff = Vector::sqrMagnitude(delta);
        B = newB;

        iteration += 1;
        //qDebug() << " Fit to shape iteration:" << iteration << diff;
        if (diff <= eps || iteration > 10000)
            end = true;
    }

    // set return variables
    trans.xt = -centTranslationCoefs.xt;
    trans.yt = -centTranslationCoefs.yt;
    rotScale.s = c.s;
    rotScale.theta = c.theta;
    return B;
}

void ActiveShapeModel::drawModelInstance(Matrix &image, Matrix &shape, double intensity)
{
    assert(intensity >= 0.0);
    assert(intensity <= 1.0);
    int n = shape.rows/2;
    QList<ActiveShapeModelPoint> &points = settings.points.points;
    assert(n == points.count());

    cv::Scalar color;
    color[0] = intensity;
    for (int i = 0; i < n; i++)
    {
        cv::Point pt1;
        cv::Point pt2;
        switch(points.at(i).type)
        {
        case pointType_begin:
            pt1.x = shape(i, 0);
            pt1.y = shape(i+n, 0);
            pt2.x = shape(points.at(i).point1, 0);
            pt2.y = shape(points.at(i).point1+n, 0);
            //qDebug() << i << pt1.x << pt1.y << "-->" << pt2.x << pt2.y;
            cv::line(image, pt1, pt2, color, 2);
            break;
        case pointType_line:
            pt1.x = shape(i, 0);
            pt1.y = shape(i+n, 0);
            pt2.x = shape(points.at(i).point2, 0);
            pt2.y = shape(points.at(i).point2+n, 0);
            //qDebug() << i << pt1.x << pt1.y << "-->" << pt2.x << pt2.y;
            cv::line(image, pt1, pt2, color, 2);
            break;
        }
    }
}
