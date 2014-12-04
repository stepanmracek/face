#include "faceCommon/linalg/ldaofpca.h"

using namespace Face::LinAlg;

LDAofPCA::LDAofPCA (const std::vector<Vector> &vectors, const std::vector<int> &classMembership, double pcaSelectionThreshold, bool debug)
{
    learn(vectors, classMembership, pcaSelectionThreshold, debug);
}

void LDAofPCA::learn(const std::vector<Vector> &vectors, const std::vector<int> &classMembership, double pcaSelectionThreshold, bool debug)
{
    // pca
    pca.learn(vectors, 0, debug);
    int oldModes = pca.getModes();
    pca.modesSelectionThreshold(pcaSelectionThreshold);
    if (debug)
        std::cout << "PCA Done; |modes| = " << pca.getModes() << " (was " << oldModes << ")" << std::endl;

    if (debug)
        std::cout << "PCA projection" << std::endl;
    std::vector<Vector> projected;
    for (unsigned int i = 0; i < vectors.size(); i++)
    {
        Vector p = pca.project(vectors[i]);
        projected.push_back(p);
    }

    // lda
    lda.learn(projected, classMembership, debug);
}

Vector LDAofPCA::project(const Vector &vector) const
{
    // pca projection
    Vector pcaProjection = pca.project(vector);

    // lda projection
    return lda.project(pcaProjection);
}
