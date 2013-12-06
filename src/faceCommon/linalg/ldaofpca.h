#ifndef LDAOFPCA_H
#define LDAOFPCA_H

#include <QVector>

#include "pca.h"
#include "lda.h"
#include "linalg/common.h"
#include "projectionbase.h"
#include "vector.h"

class LDAofPCA : public ProjectionBase
{
public:
    LDA lda;
    PCA pca;

    LDAofPCA() {}

    LDAofPCA(QVector<Vector> &vectors, QVector<int> &classMembership, double pcaSelectionThreshold = 0.98, bool debug = false);

    //LDAofPCA(const char *path);

    void learn(QVector<Vector> &vectors, QVector<int> &classMembership, double pcaSelectionThreshold = 0.98, bool debug = false);

    Vector project(const Vector &vector) const;

    Vector normalizeParams(const Vector &params) { return params; }
};

#endif // LDAOFPCA_H
