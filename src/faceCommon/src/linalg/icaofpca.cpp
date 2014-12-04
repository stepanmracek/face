#include "faceCommon/linalg/icaofpca.h"

using namespace Face::LinAlg;

void ICAofPCA::learn(const std::vector<Vector> &vectors, double pcaSelectionThreshold, int independentComponentCount, double epsICA, bool debug)
{
    pca = PCA(vectors);
    //pca.setModes(20);
    if (pcaSelectionThreshold < 1.0)
        pca.modesSelectionThreshold(pcaSelectionThreshold);

    if (debug)
        std::cout << "Projecting vectors to PCA space..." << std::endl;
    std::vector<Vector> pcaProjected;
    for (unsigned int i = 0; i < vectors.size(); i++)
    {
        Vector projected = pca.project(vectors[i]);
        pcaProjected.push_back(projected);
    }
    if (debug)
        std::cout << "...done" << std::endl;

    ica = ICA(pcaProjected, independentComponentCount, epsICA, debug);
}

ICAofPCA::ICAofPCA(const std::vector<Vector> &vectors,
                   double pcaSelectionThreshold,
                   int independentComponentCount,
                   double epsICA, bool debug)
{
    learn(vectors, pcaSelectionThreshold, independentComponentCount, epsICA, debug);
}

Vector ICAofPCA::project(const Vector &vector) const
{
    Vector projected = pca.project(vector);
    Vector result = ica.project(projected);
    return result;
}

Vector ICAofPCA::whiten(const Vector &vector) const
{
    Vector pcaProjected = pca.project(vector);
    Vector result = ica.whiten(pcaProjected);
    return result;
}

std::vector<Vector> ICAofPCA::whiten(const std::vector<Vector> &vectors) const
{
    std::vector<Vector> result;
    for (unsigned int i = 0; i < vectors.size(); i++)
    {
        Vector out = whiten(vectors[i]);
        result.push_back(out);
    }
    return result;
}

Vector ICAofPCA::backProject(const Vector &vector)
{
    Vector backProjected = ica.backProject(vector);
    Vector result = pca.backProject(backProjected);
    return result;
}
