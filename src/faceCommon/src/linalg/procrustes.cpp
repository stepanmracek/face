#include "faceCommon/linalg/procrustes.h"

#include <cmath>

using namespace Face::LinAlg;

/***
 * Procrustes analysis
 */
void Procrustes2D::procrustesAnalysis(std::vector<Vector> &vectors, bool scale, double eps, int maxIterations)
{
    std::cout << "Procrustes analysis" << std::endl;

    int count = vectors.size();
    //int len = vectors[0].rows;

    // Translate each vector to the center of origin
    centralize(vectors);

    // Choose one example as an initial estimate of the mean shape and scale so that |x| = 1.
    Vector mean = scale ? vectors[0].normalized() : vectors[0];
    Vector oldMean(mean);
    //double oldDelta = 1e300;

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
        //oldDelta = delta;

        if (iteration % 100 == 0)
            std::cout << "  iteration: " << iteration << "; delta: " << delta << std::endl;

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

TranslationCoefs Procrustes2D::centralizedTranslation(const Vector &vector)
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

void Procrustes2D::centralize(std::vector<Vector> &vectors)
{
    int n = vectors.size();
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

RotateAndScaleCoefs Procrustes2D::align(const Vector &from, const Vector &to)
{
    double referenceScale = 1.0/to.magnitude();
    Vector reference = to.mul(referenceScale);

    double vectorScale = 1.0/from.magnitude();
    Vector vector = from.mul(vectorScale);

    double s = vectorScale/referenceScale;
    double theta = getOptimalRotation(vector, reference);
    RotateAndScaleCoefs c(s, theta);

    return c;
}

Vector Procrustes2D::getMeanShape(std::vector<Vector> &vectors)
{
    int n = vectors.size();
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

cv::Point3d Procrustes3D::centralizedTranslation(const std::vector<cv::Point3d> &points)
{
    cv::Point3d mean(0,0,0);
    int numberOfPoints = points.size();
    for (int j = 0; j < numberOfPoints; j++)
    {
        mean += points[j];
    }
    mean.x /= numberOfPoints;
    mean.y /= numberOfPoints;
    mean.z /= numberOfPoints;

    cv::Point3d shift = -mean;
    return shift;
}

cv::Point3d Procrustes3D::centralizedTranslation(const Matrix &points)
{
    double meanx = cv::sum(points.col(0))[0]/points.rows;
    double meany = cv::sum(points.col(1))[0]/points.rows;
    double meanz = cv::sum(points.col(2))[0]/points.rows;

    return -cv::Point3d(meanx, meany, meanz);
}

Matrix Procrustes3D::alignRigid(std::vector<cv::Point3d> &from, const std::vector<cv::Point3d> &to)
{
    cv::Point3d centralizeFrom = centralizedTranslation(from);
    cv::Point3d centralizeTo = centralizedTranslation(to);

    std::vector<cv::Point3d> toCopy = to;
    translate(from, centralizeFrom);
    translate(toCopy, centralizeTo);

    Matrix R = getOptimalRotation(from, toCopy);
    transform(from, R);

    translate(from, -centralizeTo);
    return R;
}

Matrix Procrustes3D::getOptimalRotation(const std::vector<cv::Point3d> &from, const std::vector<cv::Point3d> &to)
{
	unsigned int n = from.size();
    if (n != to.size()) throw FACELIB_EXCEPTION("'from' and 'to' count mismatch");

    // calculate H
    Matrix H = Matrix::zeros(3, 3);
    for (unsigned int i = 0; i < n; i++)
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

Matrix Procrustes3D::getOptimalRotation(const Matrix &from, const Matrix &to)
{
    if (from.cols != 3 || to.cols != 3 || from.rows != to.rows)
        throw FACELIB_EXCEPTION("invalid input sizes");

    // calculate H
    Matrix H = Matrix::zeros(3, 3);
    for (int r = 0; r < from.rows; r++)
    {
        H += (from.row(r).t() * to.row(r));
    }

    // SVD(H)
    cv::SVD svd(H);

    // R = VU^T
    Matrix R = svd.vt.t() * svd.u.t();
    return R;
}

cv::Point3d Procrustes3D::getOptimalTranslation(const std::vector<cv::Point3d> &from, const std::vector<cv::Point3d> &to)
{
    cv::Point3d centralizedTranslationFrom = centralizedTranslation(from);
    cv::Point3d centralizedTranslationTo = centralizedTranslation(to);

    return centralizedTranslationFrom - centralizedTranslationTo;
}

void Procrustes3D::rotate(std::vector<cv::Point3d> &points, double x, double y, double z)
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

void Procrustes3D::rotate(Matrix &points, double x, double y, double z)
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

void Procrustes3D::transform(cv::Point3d &p, const Matrix &m)
{
    Matrix A = (Matrix(3,1) << p.x, p.y, p.z);
    A = m*A;
    p.x = A(0); p.y = A(1); p.z = A(2);
}


void Procrustes3D::transform(std::vector<cv::Point3d> &points, const Matrix &m)
{
    int n = points.size();
    for (int i = 0; i < n; i++)
    {
        cv::Point3d &p = points[i];
        transform(p, m);
    }
}

void Procrustes3D::transform(Matrix &points, const Matrix &m)
{
    points = points*m.t(); //.inv();
}

void Procrustes3D::inverseTransform(Matrix &points, const Matrix &m)
{
    points = points*m;
}

cv::Point3d Procrustes3D::getOptimalScale(const std::vector<cv::Point3d> &from, const std::vector<cv::Point3d> &to)
{
    cv::Point3d scaleParamsNumerator(0.0, 0.0, 0.0);
    cv::Point3d scaleParamsDenominator(0.0, 0.0, 0.0);
    int pointCount = from.size();
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

void Procrustes3D::scale(std::vector<cv::Point3d> &points, cv::Point3d scaleParams)
{
    int n = points.size();
    for (int i = 0; i < n; i++)
    {
        points[i].x = points[i].x * scaleParams.x;
        points[i].y = points[i].y * scaleParams.y;
        points[i].z = points[i].z * scaleParams.z;
    }
}

void Procrustes3D::scale(Matrix &points, cv::Point3d scaleParams)
{
    for (int r = 0; r < points.rows; r++)
    {
        points(r, 0) *= scaleParams.x;
        points(r, 1) *= scaleParams.y;
        points(r, 2) *= scaleParams.z;
    }
}

void Procrustes3D::translate(std::vector<cv::Point3d> &points, cv::Point3d shift)
{
    int n = points.size();
    for (int i = 0; i < n; i++)
    {
        points[i] += shift;
    }
}

void Procrustes3D::translate(Matrix &points, cv::Point3d shift)
{
    for (int r = 0; r < points.rows; r++)
    {
        points(r, 0) = points(r, 0) + shift.x;
        points(r, 1) = points(r, 1) + shift.y;
        points(r, 2) = points(r, 2) + shift.z;
    }
}

void Procrustes3D::applyInversedProcrustesResult(std::vector<cv::Point3d> &pointCloud, Procrustes3DResult &procrustesResult)
{
	unsigned int n = procrustesResult.rotations.size();
    if (n != procrustesResult.scaleParams.size()) throw FACELIB_EXCEPTION("invalud input");

    for (int i = n-1; i >= 0; i--)
    {
        cv::Point3d s = procrustesResult.scaleParams[i];
        cv::Point3d invScale(1.0/s.x, 1.0/s.y, 1.0/s.z);
        scale(pointCloud, invScale);

        Matrix invRot = procrustesResult.rotations[i].inv();
        transform(pointCloud, invRot);
    }
}

void Procrustes3D::applyInversedProcrustesResult(Matrix &pointCloud, Procrustes3DResult &procrustesResult)
{
	unsigned int n = procrustesResult.rotations.size();
    if (n != procrustesResult.scaleParams.size()) throw FACELIB_EXCEPTION("invalud input");

    for (int i = n-1; i >= 0; i--)
    {
        cv::Point3d s = procrustesResult.scaleParams[i];
        cv::Point3d invScale(1.0/s.x, 1.0/s.y, 1.0/s.z);
        scale(pointCloud, invScale);

        Matrix invRot = procrustesResult.rotations[i].inv();
        transform(pointCloud, invRot);
    }
}

double Procrustes3D::diff(const std::vector<cv::Point3d> &first, const std::vector<cv::Point3d> &second)
{
	unsigned int n = first.size();
    if (n != second.size()) throw FACELIB_EXCEPTION("invalud input");
    double sum = 0;
    for (unsigned int i = 0; i < n; i++)
    {
        sum += Face::LinAlg::euclideanDistance(first[i], second[i]);
    }
    return sum;
}

Procrustes3DResult Procrustes3D::SVDAlign(std::vector<std::vector<cv::Point3d> > &vectorOfPointclouds)//, bool centralize, double eps, int maxIterations)
{
    Procrustes3DResult result;

    int numberOfPointclouds = vectorOfPointclouds.size();
    if (numberOfPointclouds == 0) throw FACELIB_EXCEPTION("invalid input");
    int numberOfPoints = vectorOfPointclouds[0].size();
    if (numberOfPoints == 0) throw FACELIB_EXCEPTION("invalid input");

    //result.translations = std::vector<cv::Point3d>(numberOfPointclouds, cv::Point3d(0,0,0));
    result.scaleParams = std::vector<cv::Point3d>(numberOfPointclouds, cv::Point3d(1,1,1));
    result.rotations = std::vector<Matrix>(numberOfPointclouds, Matrix::eye(3,3));

    std::vector<cv::Point3d> mean = getMeanShape(vectorOfPointclouds);
    //double variation = getShapeVariation(vectorOfPointclouds, mean);
    //qDebug() << "SVDAlign input variation:" << variation;

    for (int i = 0; i < numberOfPointclouds; i++)
    {
        std::vector<cv::Point3d> &pointcloud = vectorOfPointclouds[i];

        Matrix R = getOptimalRotation(pointcloud, mean);
        transform(pointcloud, R);
        result.rotations[i] = R; // * result.rotations[i];
    }

    //variation = getShapeVariation(vectorOfPointclouds, mean);
    //qDebug() << "SVDAlign result variation:" << variation;

    return result;
}

std::vector<cv::Point3d> Procrustes3D::getMeanShape(const std::vector<std::vector<cv::Point3d> > &vectorOfPointclouds)
{
    int numberOfPointclouds = vectorOfPointclouds.size();
    int numberOfPoints = vectorOfPointclouds[0].size();
    std::vector<cv::Point3d> mean(numberOfPoints, cv::Point3d(0,0,0));

    for (int i = 0; i < numberOfPointclouds; i++)
    {
        const std::vector<cv::Point3d> &pointcloud = vectorOfPointclouds[i];
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

double Procrustes3D::getShapeVariation(const std::vector<std::vector<cv::Point3d> > &vectorOfPointclouds, std::vector<cv::Point3d> &mean)
{
    int numberOfPointclouds = vectorOfPointclouds.size();
    int numberOfPoints = vectorOfPointclouds[0].size();

    //std::vector<cv::Point3d> variation(numberOfPoints, cv::Point3d(0,0,0));
    double variation = 0;
    for (int i = 0; i < numberOfPointclouds; i++)
    {
        const std::vector<cv::Point3d> &pointcloud = vectorOfPointclouds[i];
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
