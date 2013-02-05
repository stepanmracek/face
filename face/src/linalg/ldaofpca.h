#ifndef LDAOFPCA_H
#define LDAOFPCA_H

#include <QVector>

#include "pca.h"
#include "lda.h"
#include "linalg/common.h"
#include "projectionbase.h"

class LDAofPCA : public ProjectionBase
{
public:
    LDA lda;
    PCA pca;

    LDAofPCA() {}

    LDAofPCA(QVector<Matrix> &vectors, QVector<int> &classMembership, double pcaSelectionThreshold = 0.98, bool debug = false);

    //LDAofPCA(const char *path);

    void learn(QVector<Matrix> &vectors, QVector<int> &classMembership, double pcaSelectionThreshold = 0.98, bool debug = false);

    Matrix project(const Matrix &vector);

    QVector<Matrix> project(const QVector<Matrix> &vector);

    Matrix normalizeParams(const Matrix &params) { return params; }
};

#endif // LDAOFPCA_H
