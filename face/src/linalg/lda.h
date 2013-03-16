#ifndef LDA_H
#define LDA_H

#include <QVector>

#include <opencv/cv.h>

#include "common.h"
#include "projectionbase.h"

class LDA : public ProjectionBase
{
public:
    Matrix Wt;
    Matrix mean;

    LDA();

    LDA(QVector<Vector> &vectors, QVector<int> &classMembership, bool debug = false);

    LDA(const char *path);

    void learn(QVector<Vector> &vectors, QVector<int> &classMembership, bool debug = false);

    Vector project(const Vector &vector);

    QVector<Vector> project(const QVector<Vector> &vectors);

    void serialize(const char *path);

    Vector normalizeParams(const Vector &params) { return params; }
};

#endif // LDA_H
