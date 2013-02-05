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

class Vector
{
public:

    static Matrix fromFile(const QString &path);

    static double sqrMagnitude(Matrix &vector);

    static double magnitude(Matrix &vector);

    static double dot(Matrix &v1, Matrix &v2);

    static Matrix &normalize(Matrix &vector);

    static Matrix normalized(Matrix &vector);

    static Matrix &mul(Matrix &vector, double value);

    static bool isZero(Matrix &vector);

    static void toFile(const Matrix &vector, const QString &path, bool append = false);

    static void toFileWithIndicies(const Matrix &vector, const QString &path, bool append = false);

    static void toFileTwoCols(Matrix &vector,const QString &path, bool append = false);

    static Matrix fromTwoColsFile(const QString &path);

    static double maxValue(Matrix &vector);

    static int maxIndex(Matrix &vector);

    static double minValue(Matrix &vector);

    static int minIndex(Matrix &vector);

    static double meanValue(Matrix &vector);

    static double stdDeviation(Matrix &vector);

    static Matrix fromQVector(QVector<double> &vec);

    static QVector<double> toQVector(Matrix &vector);

    static Matrix normalizeComponents(Matrix &vector, QVector<double> &minValues, QVector<double> &maxValues, bool fixedBounds = true);

    static Matrix normalizeComponents(Matrix &vector);

    static int maxIndex(QVector<double> &vector);

    static int minIndex(QVector<double> &vector);

    static double maxValue(QVector<double> &vector);

    static double minValue(QVector<double> &vector);

    static double meanValue(QVector<double> &vector);

    static Matrix meanVector(QVector<Matrix> &vectors);

    static double stdDeviation(QVector<double> &vector);
};

#endif // VECTORN_H
