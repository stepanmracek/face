#ifndef ICA_H
#define ICA_H

#include <QVector>
#include <QDebug>

#include "common.h"
#include "projectionbase.h"

class ICA : public ProjectionBase
{
public:

    ICA() {}

    ICA(QVector<Matrix> &vectors, int independentComponentCount = 0, double eps = 1e-10, int maxIterations = 10000, bool debug = false);

    ICA(const char *path);

    void learn(QVector<Matrix> &vectors, int independentComponentCount = 0, double eps = 1e-10, int maxIterations = 10000, bool debug = false);

    void serialize(const char *path);

    Matrix whiten(const Matrix &vector);

    Matrix project(const Matrix &vector);

    QVector<Matrix> project(const QVector<Matrix> &vectors);

    Matrix backProject(const Matrix &vector);

    int getModes() { return W.rows; }

    void setModes(int modes)
    {
        W = W.rowRange(0, modes);
    }

    Matrix normalizeParams(const Matrix &params) { return params; }

    Matrix EDETinv;
    Matrix EDET;
    Matrix mean;
    Matrix W;
};

#endif // ICA_H
