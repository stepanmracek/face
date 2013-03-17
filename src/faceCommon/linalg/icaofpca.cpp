#include "icaofpca.h"

#include <QDebug>

void ICAofPCA::learn(QVector<Vector> &vectors, double pcaSelectionThreshold, int independentComponentCount, double epsICA, bool debug)
{
    pca = PCA(vectors);
    //pca.setModes(20);
    if (pcaSelectionThreshold < 1.0)
        pca.modesSelectionThreshold(pcaSelectionThreshold);

    if (debug)
        qDebug() << "Projecting vectors to PCA space...";
    QVector<Vector> pcaProjected;
    for (int i = 0; i < vectors.count(); i++)
    {
        Vector projected = pca.project(vectors[i]);
        pcaProjected.append(projected);
    }
    if (debug)
        qDebug() << "...done";

    ica = ICA(pcaProjected, independentComponentCount, epsICA, debug);
}

ICAofPCA::ICAofPCA(QVector<Vector> &vectors,
                   double pcaSelectionThreshold,
                   int independentComponentCount,
                   double epsICA, bool debug)
{
    learn(vectors, pcaSelectionThreshold, independentComponentCount, epsICA, debug);
}

Vector ICAofPCA::project(const Vector &vector)
{
    Vector projected = pca.project(vector);
    Vector result = ica.project(projected);
    return result;
}

Vector ICAofPCA::whiten(const Vector &vector)
{
    Vector pcaProjected = pca.project(vector);
    Vector result = ica.whiten(pcaProjected);
    return result;
}

QVector<Vector> ICAofPCA::whiten(const QVector<Vector> &vectors)
{
    QVector<Vector> result;
    for (int i = 0; i < vectors.count(); i++)
    {
        Vector out = whiten(vectors[i]);
        result.append(out);
    }
    return result;
}

Vector ICAofPCA::backProject(const Vector &vector)
{
    Vector backProjected = ica.backProject(vector);
    Vector result = pca.backProject(backProjected);
    return result;
}
