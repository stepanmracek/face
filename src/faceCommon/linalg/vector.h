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

    Vector(const Matrix &m);

    Vector(const cv::MatExpr &expr);

    Vector(const Vector &src);

    Vector(const QVector<double> &vec);

    static Vector fromFile(const QString &path);

    static Vector concatenate(const QVector<Vector> &vectors);

    double sqrMagnitude() const;

    double magnitude() const;

    static double dot(const Vector &v1, const Vector &v2);

    Vector &normalize();

    Vector normalized() const;

    Vector mul(double value) const;

    Vector mul(const Vector &other) const;

    bool isZero() const;

    void toFile(const QString &path, bool append = false) const;

    void toFileWithIndicies(const QString &path, bool append = false) const;

    void toFileTwoCols(const QString &path, bool append = false) const;

    static Vector fromTwoColsFile(const QString &path);

    double maxValue() const;

    int maxIndex() const;

    int maxIndex(int from, int to) const;

    double minValue() const;

    int minIndex() const;

    double meanValue() const;

    double stdDeviation() const;

    QVector<double> toQVector() const;

    Vector normalizeComponents(const QVector<double> &minValues, const QVector<double> &maxValues, bool fixedBounds = true) const;

    Vector normalizeComponents() const;

    /*static int maxIndex(QVector<double> &vector);

    static int minIndex(QVector<double> &vector);

    static double maxValue(QVector<double> &vector);

    static double minValue(QVector<double> &vector);

    static double meanValue(QVector<double> &vector);*/

    static Vector meanVector(const QVector<Vector> &vectors);

    /*static double stdDeviation(QVector<double> &vector);*/

    Vector smooth(int kernelSize) const;

    bool containsNan() const;
};

#endif // VECTORN_H
