#include "procrustes.h"

//#include <cassert>
#include <cmath>

/***
 * Procrustes analysis
 */
void Procrustes2D::procrustesAnalysis(QVector<Vector> &vectors, bool scale, double eps, int maxIterations)
{
    qDebug() << "Procrustes analysis";

    int count = vectors.count();
    assert(count > 0);

    int len = vectors[0].rows;
    assert(len > 0);
    assert(len % 2 == 0);
    //int n = len / 2;

    // Translate each vector to the center of origin
    centralize(vectors);

    // Choose one example as an initial estimate of the mean shape and scale so that |x| = 1.
    Vector mean = scale ? vectors[0].normalized() : vectors[0];
    Vector oldMean(mean);
    double oldDelta = 1e300;

    int iteration = 1;
    bool end = false;
    while (!end)
    {
        // Align all the shapes with the current estimate of the mean shape.
        for (int i = 0; i < count; i++)
        {
            if (scale)
            {
                RotateAndScaleCoefs c = align(vectors[i], mean);
                rotateAndScale(vectors[i], c);
            }
            else
            {
                double theta = getOptimalRotation(vectors[i], mean);
                RotateAndScaleCoefs c; c.theta = theta;
                rotateAndScale(vectors[i], c);
            }
        }

        // Re-estimate mean from aligned shapes
        mean = getMeanShape(vectors);
        /*Matrix::zeros(mean.rows, mean.cols, CV_64F);
        for (int i = 0; i < count; i++)
        {
            mean = mean + vectors[i];
        }
        Vector::mul(mean, 1.0/count);*/

        // Apply constraints on the current estimate of the mean
        if (scale)
            mean.normalize();

        // If not converged, iterate
        Vector diff = mean-oldMean;
        double delta = diff.sqrMagnitude();

        if (delta <= eps || iteration > maxIterations)
            end = true;

        oldMean = Vector(mean);
        oldDelta = delta;

        if (iteration % 100 == 0)
            qDebug() << " iteration:" << iteration << "; delta:" << delta;

        iteration += 1;
    }
}

void Procrustes2D::translate(Vector &vector, TranslationCoefs &coefs)
{
    int n = vector.rows/2;
    for (int i = 0; i < n; i++)
    {
        vector(i, 0) += coefs.xt;
        vector(i+n, 0) += coefs.yt;
    }
}

TranslationCoefs Procrustes2D::centralizedTranslation(Vector &vector)
{
    double meanx = 0.0;
    double meany = 0.0;
    int n = vector.rows/2;

    for (int i = 0; i < n; i++)
    {
        meanx += vector(i, 0);
        meany += vector(i+n, 0);
    }
    meanx /= n;
    meany /= n;

    TranslationCoefs c;
    c.xt = -meanx;
    c.yt = -meany;
    return c;
}

void Procrustes2D::centralize(Vector &vector)
{
    TranslationCoefs c = centralizedTranslation(vector);
    translate(vector, c);
}

void Procrustes2D::centralize(QVector<Vector> &vectors)
{
	int n = vectors.count();
	for (int i = 0; i < n; i++)
		centralize(vectors[i]);
}

void Procrustes2D::rotateAndScale(Vector &vector, RotateAndScaleCoefs &coefs)
{
    double sint = coefs.s * sin(coefs.theta);
    double cost = coefs.s * cos(coefs.theta);

    int n = vector.rows/2;
    for (int i = 0; i < n; i++)
    {
        double oldx = vector(i, 0);
        double oldy = vector(i+n, 0);
        double x = cost*oldx - sint*oldy;
        double y = sint*oldx + cost*oldy;

        vector(i, 0) = x;
        vector(i+n, 0) = y;
    }
}

void Procrustes2D::transformate(Vector &vector, TransformationCoefs &coefs)
{
    int n = vector.rows/2;
    for (int i = 0; i < n; i++)
    {
        double oldx = vector(i, 0);
        double oldy = vector(i+n, 0);
        double x = coefs.a * oldx + -coefs.b * oldy;
        double y = coefs.b * oldx + coefs.a * oldy;

        vector(i, 0) = x;
        vector(i+n, 0) = y;
    }
}

double Procrustes2D::getOptimalRotation(Vector &from, Vector &to)
{
    int n = from.rows/2;
    double numerator = 0.0;
    double denumerator = 0.0;
    for (int i = 0; i < n; i++)
    {
        numerator += (from(i) * to(n+i) - from(n+i) * to(i));
        denumerator += (from(i) * to(i) + from(n+i) * to(n+i));
    }

    return atan(numerator/denumerator);
}

RotateAndScaleCoefs Procrustes2D::align(Vector &from, Vector &to)
{
    Vector reference(to);
    double referenceScale = 1.0/reference.magnitude();
    reference.mul(referenceScale);

    Vector vector(from);
    double vectorScale = 1.0/vector.magnitude();
    vector.mul(vectorScale);

    /*int n = vector.rows/2;
    double numerator = 0.0;
    double denumerator = 0.0;
    for (int i = 0; i < n; i++)
    {
        numerator += (vector(i, 0) * reference(n+i, 0) - vector(n+i, 0) * reference(i, 0));
        denumerator += (vector(i, 0) * reference(i, 0) + vector(n+i, 0) * reference(n+i, 0));
    }*/

    double s = vectorScale/referenceScale;
    //double theta = atan(numerator/denumerator);
    double theta = getOptimalRotation(vector, reference);
    RotateAndScaleCoefs c(s, theta);

    //reference.mult(1.0/referenceScale);
    //vector.mult(1.0/vectorScale);

    return c;
}

Vector Procrustes2D::getMeanShape(QVector<Vector> &vectors)
{
	int n = vectors.count();
    Vector mean(vectors[0].rows);
	for (int i = 0; i < n; i++)
	{
		mean += vectors[0];
	}
    mean = mean / ((double)n);
	return mean;
}

/*TransformationCoefs Procrustes::AlignAlt(Vector &from, Vector &to)
{
    int n = from.len()/2;
    double vecSqr = from.sqrMagnitude();

    double a = (from*to)/vecSqr;
    double b = 0.0;
    for (int i = 0; i < n; i++)
        b = b+(from.data.at(i) * to.data.at(i+n) + from.data.at(i+n) * to.data.at(i));
    b = b/vecSqr;

    TransformationCoefs coefs;
    coefs.a = a;
    coefs.b = b;
    return coefs;
}*/

// ------------------------------------------------------------------------------

cv::Point3d Procrustes3D::centralizedTranslation(const QVector<cv::Point3d> &shape)
{
    cv::Point3d mean(0,0,0);
    int numberOfPoints = shape.count();
    for (int j = 0; j < numberOfPoints; j++)
    {
        mean += shape[j];
    }
    mean.x /= numberOfPoints;
    mean.y /= numberOfPoints;
    mean.z /= numberOfPoints;

    cv::Point3d shift = -mean;
    return shift;
}

Matrix Procrustes3D::alignRigid(QVector<cv::Point3d> &from, QVector<cv::Point3d> &to, bool centralize)
{
    int n = from.count();
    assert(n == to.count());

    // centralize
    if (centralize)
    {
        cv::Point3d cumulativeA(0, 0, 0);
        cv::Point3d cumulativeB(0, 0, 0);
        for (int i = 0; i < n; i++)
        {
            cumulativeA += from[i];
            cumulativeB += to[i];
        }
        cumulativeA.x = cumulativeA.x/n;
        cumulativeA.y = cumulativeA.y/n;
        cumulativeA.z = cumulativeA.z/n;
        cumulativeB.x = cumulativeB.x/n;
        cumulativeB.y = cumulativeB.y/n;
        cumulativeB.z = cumulativeB.z/n;

        for (int i = 0; i < n; i++)
        {
            from[i] = from[i] - cumulativeA;
            to[i] = to[i] - cumulativeB;
        }
    }

    Matrix R = getOptimalRotation(from, to);
    transform(from, R);
    return R;
}

Matrix Procrustes3D::getOptimalRotation(QVector<cv::Point3d> &from, QVector<cv::Point3d> &to)
{
    int n = from.count();
    assert(n == to.count());

    // calculate H
    Matrix H = Matrix::zeros(3, 3);
    for (int i = 0; i < n; i++)
    {
        Matrix A = (Matrix(3,1) << from[i].x, from[i].y, from[i].z);
        Matrix B = (Matrix(3,1) << to[i].x, to[i].y, to[i].z);
        H += (A * B.t());
    }

    // SVD(H)
    cv::SVD svd(H);

    // R = VU^T
    Matrix R = svd.vt.t() * svd.u.t();
    return R;
}

cv::Point3d Procrustes3D::getOptimalTranslation(QVector<cv::Point3d> &from, QVector<cv::Point3d> &to)
{
    cv::Point3d centralizedTranslationFrom = centralizedTranslation(from);
    cv::Point3d centralizedTranslationTo = centralizedTranslation(to);

    return centralizedTranslationFrom - centralizedTranslationTo;
}

void Procrustes3D::rotate(QVector<cv::Point3d> &points, double x, double y, double z)
{
        Matrix Rx = (Matrix(3,3) <<
                     1, 0, 0,
                     0, cos(x), -sin(x),
                     0, sin(x), cos(x));
        Matrix Ry = (Matrix(3,3) <<
                     cos(y), 0, sin(y),
                     0, 1, 0,
                     -sin(y), 0, cos(y));
        Matrix Rz = (Matrix(3,3) <<
                     cos(z), -sin(z), 0,
                     sin(z), cos(z), 0,
                     0, 0, 1);
        Matrix R = Rx*Ry*Rz;

        transform(points, R);
}

void Procrustes3D::transform(cv::Point3d &p, Matrix &m)
{
    Matrix A = (Matrix(3,1) << p.x, p.y, p.z);
    A = m*A;
    p.x = A(0); p.y = A(1); p.z = A(2);
}


void Procrustes3D::transform(QVector<cv::Point3d> &points, Matrix &m)
{
    int n = points.count();
    for (int i = 0; i < n; i++)
    {
        cv::Point3d &p = points[i];
        transform(p, m);
    }
}

cv::Point3d Procrustes3D::getOptimalScale(const QVector<cv::Point3d> &from, const QVector<cv::Point3d> &to)
{
    cv::Point3d scaleParamsNumerator(0.0, 0.0, 0.0);
    cv::Point3d scaleParamsDenominator(0.0, 0.0, 0.0);
    int pointCount = from.count();
    for (int i = 0; i < pointCount; i++)
    {
        scaleParamsNumerator.x += from[i].x * to[i].x;
        scaleParamsNumerator.y += from[i].y * to[i].y;
        scaleParamsNumerator.z += from[i].z * to[i].z;

        scaleParamsDenominator.x += from[i].x * from[i].x;
        scaleParamsDenominator.y += from[i].y * from[i].y;
        scaleParamsDenominator.z += from[i].z * from[i].z;
    }
    cv::Point3d scaleParams(scaleParamsNumerator.x/scaleParamsDenominator.x,
                            scaleParamsNumerator.y/scaleParamsDenominator.y,
                            scaleParamsNumerator.z/scaleParamsDenominator.z);
    return scaleParams;

}

void Procrustes3D::scale(QVector<cv::Point3d> &points, cv::Point3d scaleParams)
{
    int n = points.count();
    for (int i = 0; i < n; i++)
    {
        points[i].x = points[i].x * scaleParams.x;
        points[i].y = points[i].y * scaleParams.y;
        points[i].z = points[i].z * scaleParams.z;
    }
}

void Procrustes3D::translate(QVector<cv::Point3d> &points, cv::Point3d shift)
{
    int n = points.count();
    for (int i = 0; i < n; i++)
    {
        points[i] += shift;
    }
}

void Procrustes3D::applyInversedProcrustesResult(QVector<cv::Point3d> &pointCloud, Procrustes3DResult &procrustesResult)
{
    int n = procrustesResult.rotations.count();
    assert(n == procrustesResult.scaleParams.count());

    for (int i = n-1; i >= 0; i--)
    {
        cv::Point3d s = procrustesResult.scaleParams[i];
        cv::Point3d invScale(1.0/s.x, 1.0/s.y, 1.0/s.z);
        scale(pointCloud, invScale);

        Matrix invRot = procrustesResult.rotations[i].inv();
        transform(pointCloud, invRot);
    }
}

double Procrustes3D::diff(QVector<cv::Point3d> &first, QVector<cv::Point3d> &second)
{
    int n = first.count();
    assert(n == second.count());
    double sum = 0;
    for (int i = 0; i < n; i++)
    {
        sum += euclideanDistance(first[i], second[i]);
    }
    return sum;
}

Procrustes3DResult Procrustes3D::SVDAlign(QVector<QVector<cv::Point3d> > &vectorOfPointclouds)//, bool centralize, double eps, int maxIterations)
{
    Procrustes3DResult result;

    int numberOfPointclouds = vectorOfPointclouds.count();
    assert(numberOfPointclouds > 0);
    int numberOfPoints = vectorOfPointclouds[0].count();
    assert(numberOfPoints > 0);

    //result.translations = QVector<cv::Point3d>(numberOfPointclouds, cv::Point3d(0,0,0));
    result.scaleParams = QVector<cv::Point3d>(numberOfPointclouds, cv::Point3d(1,1,1));
    result.rotations = QVector<Matrix>(numberOfPointclouds, Matrix::eye(3,3));

    QVector<cv::Point3d> mean = getMeanShape(vectorOfPointclouds);
    //double variation = getShapeVariation(vectorOfPointclouds, mean);
    //qDebug() << "SVDAlign input variation:" << variation;

    for (int i = 0; i < numberOfPointclouds; i++)
    {
        QVector<cv::Point3d> &pointcloud = vectorOfPointclouds[i];

        Matrix R = getOptimalRotation(pointcloud, mean);
        transform(pointcloud, R);
        result.rotations[i] = R; // * result.rotations[i];
    }

    //variation = getShapeVariation(vectorOfPointclouds, mean);
    //qDebug() << "SVDAlign result variation:" << variation;

    return result;
}

QVector<cv::Point3d> Procrustes3D::getMeanShape(QVector<QVector<cv::Point3d> > &vectorOfPointclouds)
{
    int numberOfPointclouds = vectorOfPointclouds.count();
    int numberOfPoints = vectorOfPointclouds[0].count();
    QVector<cv::Point3d> mean(numberOfPoints, cv::Point3d(0,0,0));

    for (int i = 0; i < numberOfPointclouds; i++)
    {
        QVector<cv::Point3d> &pointcloud = vectorOfPointclouds[i];
        for (int j = 0; j < numberOfPoints; j++)
        {
            mean[j] = mean[j] + pointcloud[j];

            /*if (j == 0)
                qDebug() << mean[j].x << mean[j].y << mean[j].z;*/
        }
    }

    for (int j = 0; j < numberOfPoints; j++)
    {
        mean[j].x /= numberOfPointclouds;
        mean[j].y /= numberOfPointclouds;
        mean[j].z /= numberOfPointclouds;
    }

    return mean;
}

double Procrustes3D::getShapeVariation(QVector<QVector<cv::Point3d> > &vectorOfPointclouds, QVector<cv::Point3d> &mean)
{
    int numberOfPointclouds = vectorOfPointclouds.count();
    int numberOfPoints = vectorOfPointclouds[0].count();

    //QVector<cv::Point3d> variation(numberOfPoints, cv::Point3d(0,0,0));
    double variation = 0;
    for (int i = 0; i < numberOfPointclouds; i++)
    {
        QVector<cv::Point3d> &pointcloud = vectorOfPointclouds[i];
        for (int j = 0; j < numberOfPoints; j++)
        {
            cv::Point3d diff = mean[j] - pointcloud[j];
            variation += (diff.x*diff.x);
            variation += (diff.y*diff.y);
            variation += (diff.z*diff.z);
        }
    }

    variation = variation / (3*numberOfPointclouds*numberOfPoints);
    return variation;
}
