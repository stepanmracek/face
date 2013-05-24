#ifndef ICA_H
#define ICA_H

#include <QVector>
#include <QDebug>

#include "common.h"
#include "projectionbase.h"
#include "vector.h"

class ICA : public ProjectionBase
{
public:

    ICA() {}

    ICA(QVector<Vector> &vectors, int independentComponentCount = 0, double eps = 1e-10, int maxIterations = 10000, bool debug = false);

    ICA(const char *path);

    void learn(QVector<Vector> &vectors, int independentComponentCount = 0, double eps = 1e-10, int maxIterations = 10000, bool debug = false);

    void serialize(const char *path);

    Vector whiten(const Vector &vector) const;

    Vector project(const Vector &vector) const;

    Vector backProject(const Vector &vector) const;

    int getModes() { return W.rows; }

    void setModes(int modes)
    {
        W = W.rowRange(0, modes);
    }

    Vector normalizeParams(const Vector &params) { return params; }

    Matrix EDETinv;
    Matrix EDET;
    Matrix mean;
    Matrix W;
};

#endif // ICA_H
