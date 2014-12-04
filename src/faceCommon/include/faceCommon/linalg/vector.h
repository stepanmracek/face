#ifndef VECTORN_H
#define VECTORN_H

#include <cmath>
#include <opencv/cv.h>

#include "common.h"

namespace Face {
namespace LinAlg {

class Vector : public Matrix
{
public:

    Vector() { }

    Vector(int size);

    Vector(int size, double values);

    Vector(const Matrix &m);

    Vector(const cv::MatExpr &expr);

    Vector(const Vector &src);

    Vector(const std::vector<double> &vec);

    static Vector fromFile(const std::string &path);

    static Vector concatenate(const std::vector<Vector> &vectors);

    double sqrMagnitude() const;

    double magnitude() const;

    static double dot(const Vector &v1, const Vector &v2);

    Vector &normalize();

    Vector normalized() const;

    Vector mul(double value) const;

    Vector mul(const Vector &other) const;

    bool isZero() const;

    void toFile(const std::string &path, bool append = false) const;

    void toFileWithIndicies(const std::string &path, bool append = false) const;

    void toFileTwoCols(const std::string &path, bool append = false) const;

    static Vector fromTwoColsFile(const std::string &path);

    double maxValue() const;

    int maxIndex() const;

    int maxIndex(int from, int to) const;

    double minValue() const;

    int minIndex() const;

    double meanValue() const;

    double stdDeviation() const;

    std::vector<double> toStdVector() const;

    Vector normalizeComponents(const std::vector<double> &minValues, const std::vector<double> &maxValues, bool fixedBounds = true) const;

    Vector normalizeComponents() const;

    static Vector meanVector(const std::vector<Vector> &vectors);

    Vector smooth(int kernelSize) const;

    bool containsNan() const;
};

}
}

#endif // VECTORN_H
