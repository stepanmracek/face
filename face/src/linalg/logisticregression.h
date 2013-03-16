#ifndef LOGISTICREGRESSION_H
#define LOGISTICREGRESSION_H

#include "common.h"
#include "vector.h"

class LogisticRegression
{
private:
    Vector prependOne(Vector &in);

public:
    LogisticRegression() {}
    LogisticRegression(QVector<Vector> &data, QVector<int> &classLabels);
    void learn(QVector<Vector> &data, QVector<int> &classLabels);

    Matrix w;

    double sigma(double x);
    Matrix sigma(Matrix x);

    Matrix createDesignMatrix(QVector<Vector> &data);

    double classify(Vector &x);
};

#endif // LOGISTICREGRESSION_H
