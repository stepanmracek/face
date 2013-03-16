#ifndef VECTORN_H
#define VECTORN_H

#include <QVector>
#include <QFile>
#include <QTextStream>
#include <QDebug>

#include <cmath>
#include <cassert>
#include <opencv/cv.h>

#include "common.h"

class Vector : public Matrix
{
public:

    Vector() { }

    Vector(int size);

    Vector(Matrix &m);

    Vector(Vector &src);

    Vector(Vector src);

    //Vector(cv::MatExpr &expr);

    Vector(QVector<double> &vec);

    static Vector fromFile(const QString &path);

    double sqrMagnitude();

    double magnitude();

    static double dot(Vector &v1, Vector &v2);

    Vector &normalize();

    Vector normalized();

    Vector &mul(double value);

    Vector &mul(Vector &other);

    bool isZero();

    void toFile(const QString &path, bool append = false);

    void toFileWithIndicies(const QString &path, bool append = false);

    void toFileTwoCols(const QString &path, bool append = false);

    static Vector fromTwoColsFile(const QString &path);

    double maxValue();

    int maxIndex();

    int maxIndex(int from, int to);

    double minValue();

    int minIndex();

    double meanValue();

    double stdDeviation();

    QVector<double> toQVector();

    Vector normalizeComponents(QVector<double> &minValues, QVector<double> &maxValues, bool fixedBounds = true);

    Vector normalizeComponents();

    /*static int maxIndex(QVector<double> &vector);

    static int minIndex(QVector<double> &vector);

    static double maxValue(QVector<double> &vector);

    static double minValue(QVector<double> &vector);

    static double meanValue(QVector<double> &vector);*/

    static Vector meanVector(QVector<Vector> &vectors);

    /*static double stdDeviation(QVector<double> &vector);*/

    Vector smooth(int kernelSize);
};

#endif // VECTORN_H
