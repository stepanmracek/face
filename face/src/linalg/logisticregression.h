#ifndef LOGISTICREGRESSION_H
#define LOGISTICREGRESSION_H

#include "common.h"

class LogisticRegression
{
private:
    Matrix prependOne(Matrix &in);

public:
    LogisticRegression() {}
    LogisticRegression(QVector<Matrix> &data, QVector<int> &classLabels);
    void learn(QVector<Matrix> &data, QVector<int> &classLabels);

    Matrix w;

    double sigma(double x);
    Matrix sigma(Matrix x);

    Matrix createDesignMatrix(QVector<Matrix> &data);

    double classify(Matrix x);
};

#endif // LOGISTICREGRESSION_H
