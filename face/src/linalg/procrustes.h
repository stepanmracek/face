#ifndef PROCRUSTES_H
#define PROCRUSTES_H

#include <QVector>

#include <opencv/cv.h>
#include <climits>

#include "linalg/vector.h"
#include "common.h"

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
    QVector<cv::Point3d> translations;
    QVector<cv::Point3d> scaleParams;
    QVector<Matrix> rotations;
};

class Procrustes
{
public:
    static void procrustesAnalysis(QVector<Matrix> &vectors, bool scale = true,
    							   double eps = 0, int maxIterations = INT_MAX);

    static double getOptimalRotation(Matrix &from, Matrix &to);

    static RotateAndScaleCoefs align(Matrix &from, Matrix &to);

    static void rotateAndScale(Matrix &vector, RotateAndScaleCoefs &coefs);

    static void transformate(Matrix &vector, TransformationCoefs &coefs);

    static void translate(Matrix &vector, TranslationCoefs &coefs);

    static TranslationCoefs centralizedTranslation(Matrix &vector);

    static void centralize(Matrix &vector);

    static void centralize(QVector<Matrix> &vectors);

    static cv::Point3d centralizedTranslation(const QVector<cv::Point3d> &shape);

    static Matrix getMeanShape(QVector<Matrix> &vectors);

    static Matrix alignRigid(QVector<cv::Point3d> &from, QVector<cv::Point3d> &to, bool centralize);

    static Procrustes3DResult SVDAlign(QVector<QVector<cv::Point3d> > &vectorOfPointclouds); //, bool centralize, double eps = 0, int maxIterations = INT_MAX);

    static Matrix getOptimalRotation(QVector<cv::Point3d> &from, QVector<cv::Point3d> &to);

    static cv::Point3d getOptimalScale(const QVector<cv::Point3d> &from, const QVector<cv::Point3d> &to);

    static void transform(cv::Point3d &p, Matrix &m);

    static void transform(QVector<cv::Point3d> &points, Matrix &m);

    static QVector<cv::Point3d> getMeanShape(QVector<QVector<cv::Point3d> > &vectorOfPointclouds);

    static void scale(QVector<cv::Point3d> &points, cv::Point3d scaleParams);

    static void translate(QVector<cv::Point3d> &points, cv::Point3d shift);

    static double getShapeVariation(QVector<QVector<cv::Point3d> > &vectorOfPointclouds, QVector<cv::Point3d> &mean);
};

#endif // PROCRUSTES_H
