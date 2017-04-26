#pragma once

#include <opencv/cv.h>
#include <climits>

#include "vector.h"
#include "common.h"

namespace Face {
namespace LinAlg {

struct RotateAndScaleCoefs
{
    RotateAndScaleCoefs()
    {
        s = 1;
        theta = 0;
    }

    RotateAndScaleCoefs(double _s, double _theta) : s(_s), theta(_theta) {}

    double s;
    double theta;
};

struct TranslationCoefs
{
    TranslationCoefs()
    {
        xt = 0.0;
        yt = 0.0;
    }

    TranslationCoefs(double _xt, double _yt) : xt(_xt), yt(_yt) {}

    double xt;
    double yt;
};

struct TransformationCoefs
{
    TransformationCoefs()
    {
        a = 1;
        b = 0;
    }

    TransformationCoefs(double _a, double _b) : a(_a), b(_b) {}

    double a;
    double b;
};

struct Procrustes3DResult
{
    std::vector<cv::Point3d> preTranslations;
    std::vector<cv::Point3d> scaleParams;
    std::vector<Matrix> rotations;
    std::vector<cv::Point3d> postTranslations;
};

class Procrustes2D
{
public:
    static void procrustesAnalysis(std::vector<Vector> &vectors, bool scale = true,
    							   double eps = 0, int maxIterations = INT_MAX);

    static double getOptimalRotation(Vector &from, Vector &to);

    static RotateAndScaleCoefs align(const Vector &from, const Vector &to);

    static void rotateAndScale(Vector &vector, RotateAndScaleCoefs &coefs);

    static void transformate(Vector &vector, TransformationCoefs &coefs);

    static void translate(Vector &vector, TranslationCoefs &coefs);

    static TranslationCoefs centralizedTranslation(const Vector &vector);

    static void centralize(Vector &vector);

    static void centralize(std::vector<Vector> &vectors);

    static Vector getMeanShape(std::vector<Vector> &vectors);
};

class FACECOMMON_EXPORTS Procrustes3D
{
public:
    static cv::Point3d centralizedTranslation(const std::vector<cv::Point3d> &points);

    static cv::Point3d centralizedTranslation(const Matrix &points);

    static Matrix alignRigid(std::vector<cv::Point3d> &from, const std::vector<cv::Point3d> &to);

    static Procrustes3DResult SVDAlign(std::vector<std::vector<cv::Point3d> > &vectorOfPointclouds); //, bool centralize, double eps = 0, int maxIterations = INT_MAX);

    static Matrix getOptimalRotation(const std::vector<cv::Point3d> &from, const std::vector<cv::Point3d> &to);

    static Matrix getOptimalRotation(const Matrix &from, const Matrix &to);

    static cv::Point3d getOptimalTranslation(const std::vector<cv::Point3d> &from, const std::vector<cv::Point3d> &to);

    static cv::Point3d getOptimalScale(const std::vector<cv::Point3d> &from, const std::vector<cv::Point3d> &to);

    static void rotate(std::vector<cv::Point3d> &points, double x, double y, double z);

    static void rotate(Matrix &points, double x, double y, double z);

    static void transform(cv::Point3d &p, Matrix const &m);

    static void transform(std::vector<cv::Point3d> &points, const Matrix &m);

    static void transform(Matrix &points, const Matrix &m);

    static void inverseTransform(Matrix &points, const Matrix &m);

    static std::vector<cv::Point3d> getMeanShape(const std::vector<std::vector<cv::Point3d> > &vectorOfPointclouds);

    static void scale(std::vector<cv::Point3d> &points, cv::Point3d scaleParams);

    static void scale(Matrix &points, cv::Point3d scaleParams);

    static void translate(Matrix &points, cv::Point3d shift);

    static void translate(std::vector<cv::Point3d> &points, cv::Point3d shift);

    static double getShapeVariation(const std::vector<std::vector<cv::Point3d> > &vectorOfPointclouds, std::vector<cv::Point3d> &mean);

    static void applyInversedProcrustesResult(std::vector<cv::Point3d> &pointCloud, Procrustes3DResult &procrustesResult);

    static void applyInversedProcrustesResult(Matrix &pointCloud, Procrustes3DResult &procrustesResult);

    static double diff(const std::vector<cv::Point3d> &first, const std::vector<cv::Point3d> &second);
};

}
}
