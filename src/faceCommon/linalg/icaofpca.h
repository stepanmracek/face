#ifndef ICAOFPCA_H
#define ICAOFPCA_H

#include <QVector>

#include "ica.h"
#include "pca.h"
#include "linalg/vector.h"
#include "projectionbase.h"

class ICAofPCA : public ProjectionBase
{
public:
    ICA ica;
    PCA pca;

    ICAofPCA() {}

    ICAofPCA(QVector<Vector> &vectors,
             double pcaSelectionThreshold = 0.98,
             int independentComponentCount = 0,
             double epsICA = 1e-10,
             bool debug = false);


    void learn(QVector<Vector> &vectors,
               double pcaSelectionThreshold = 0.98,
               int independentComponentCount = 0,
               double epsICA = 1e-10,
               bool debug = false);

    Vector project(const Vector &vector) const;

    Vector whiten(const Vector &vector) const;

    QVector<Vector> whiten(const QVector<Vector> &vectors) const;

    Vector backProject(const Vector &vector);

    Vector normalizeParams(const Vector &params) { return params; }
};

#endif // ICAOFPCA_H
