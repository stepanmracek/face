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

    ICAofPCA(QVector<Matrix> &vectors,
             double pcaSelectionThreshold = 0.98,
             int independentComponentCount = 0,
             double epsICA = 1e-10,
             bool debug = false);


    void learn(QVector<Matrix> &vectors,
               double pcaSelectionThreshold = 0.98,
               int independentComponentCount = 0,
               double epsICA = 1e-10,
               bool debug = false);

    Matrix project(const Matrix &vector);

    QVector<Matrix> project(const QVector<Matrix> &vector);

    Matrix whiten(const Matrix &vector);

    QVector<Matrix> whiten(const QVector<Matrix> &vectors);

    Matrix backProject(const Matrix &vector);

    Matrix normalizeParams(const Matrix &params) { return params; }
};

#endif // ICAOFPCA_H
