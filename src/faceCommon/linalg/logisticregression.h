#ifndef LOGISTICREGRESSION_H
#define LOGISTICREGRESSION_H

#include "common.h"
#include "vector.h"

class LogisticRegression
{
private:
    Vector prependOne(const Vector &in) const;

public:
    LogisticRegression() {}
    LogisticRegression(const QVector<Vector> &data, const QVector<int> &classLabels);
    void learn(const QVector<Vector> &data, const QVector<int> &classLabels);

    Matrix w;

    double sigma(double x) const;
    //Matrix sigma(const Matrix &x) const;

    Matrix createDesignMatrix(const QVector<Vector> &data) const;

    double classify(const Vector &x) const;
};

#endif // LOGISTICREGRESSION_H
