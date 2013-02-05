#include "icaofpca.h"

#include <QDebug>

void ICAofPCA::learn(QVector<Matrix> &vectors, double pcaSelectionThreshold, int independentComponentCount, double epsICA, bool debug)
{
    pca = PCA(vectors);
    //pca.setModes(20);
    if (pcaSelectionThreshold < 1.0)
        pca.modesSelectionThreshold(pcaSelectionThreshold);

    if (debug)
        qDebug() << "Projecting vectors to PCA space...";
    QVector<Matrix> pcaProjected;
    for (int i = 0; i < vectors.count(); i++)
    {
        Matrix projected = pca.project(vectors[i]);
        pcaProjected.append(projected);
    }
    if (debug)
        qDebug() << "...done";

    ica = ICA(pcaProjected, independentComponentCount, epsICA, debug);
}

ICAofPCA::ICAofPCA(QVector<Matrix> &vectors,
                   double pcaSelectionThreshold,
                   int independentComponentCount,
                   double epsICA, bool debug)
{
    learn(vectors, pcaSelectionThreshold, independentComponentCount, epsICA, debug);
}

Matrix ICAofPCA::project(const Matrix &vector)
{
    Matrix projected = pca.project(vector);
    Matrix result = ica.project(projected);
    return result;
}

QVector<Matrix> ICAofPCA::project(const QVector<Matrix> &vectors)
{
    QVector<Matrix> result;
    for (int i = 0; i < vectors.count(); i++)
    {
        Matrix out = project(vectors[i]);
        result.append(out);
    }
    return result;
}

Matrix ICAofPCA::whiten(const Matrix &vector)
{
    Matrix pcaProjected = pca.project(vector);
    Matrix result = ica.whiten(pcaProjected);
    return result;
}

QVector<Matrix> ICAofPCA::whiten(const QVector<Matrix> &vectors)
{
    QVector<Matrix> result;
    for (int i = 0; i < vectors.count(); i++)
    {
        Matrix out = whiten(vectors[i]);
        result.append(out);
    }
    return result;
}

Matrix ICAofPCA::backProject(const Matrix &vector)
{
    Matrix backProjected = ica.backProject(vector);
    Matrix result = pca.backProject(backProjected);
    return result;
}
