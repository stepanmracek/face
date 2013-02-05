#include "ldaofpca.h"

#include <QDebug>

LDAofPCA::LDAofPCA (QVector<Matrix> &vectors, QVector<int> &classMembership, double pcaSelectionThreshold, bool debug)
{
    learn(vectors, classMembership, pcaSelectionThreshold, debug);
}

void LDAofPCA::learn(QVector<Matrix> &vectors, QVector<int> &classMembership, double pcaSelectionThreshold, bool debug)
{
    // pca
    pca.learn(vectors, 0, debug);
    int oldModes = pca.getModes();
    pca.modesSelectionThreshold(pcaSelectionThreshold);
    if (debug)
        qDebug() << "PCA Done; |modes| =" << pca.getModes() << "(was" << oldModes << ")";

    if (debug)
        qDebug() << "PCA projection";
    QVector<Matrix> projected;
    for (int i = 0; i < vectors.count(); i++)
    {
        Matrix p = pca.project(vectors[i]);
        projected.append(p);
    }

    // lda
    lda.learn(projected, classMembership, debug);
}

Matrix LDAofPCA::project(const Matrix &vector)
{
    // pca projection
    Matrix pcaProjection = pca.project(vector);

    // lda projection
    return lda.project(pcaProjection);
}

QVector<Matrix> LDAofPCA::project(const QVector<Matrix> &vectors)
{
    QVector<Matrix> result;
    for (int i = 0; i < vectors.count(); i++)
    {
        Matrix out = project(vectors[i]);
        result.append(out);
    }
    return result;
}
