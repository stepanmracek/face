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

    LDA(QVector<Matrix> &vectors, QVector<int> &classMembership, bool debug = false);

    LDA(const char *path);

    void learn(QVector<Matrix> &vectors, QVector<int> &classMembership, bool debug = false);

    Matrix project(const Matrix &vector);

    QVector<Matrix> project(const QVector<Matrix> &vectors);

    void serialize(const char *path);

    Matrix normalizeParams(const Matrix &params) { return params; }
};

#endif // LDA_H
