#pragma once

#include "pca.h"
#include "lda.h"
#include "common.h"
#include "projectionbase.h"
#include "vector.h"

namespace Face {
namespace LinAlg {

class LDAofPCA : public ProjectionBase
{
public:
    LDA lda;
    PCA pca;

    LDAofPCA() {}

    LDAofPCA(const std::vector<Vector> &vectors, const std::vector<int> &classMembership, double pcaSelectionThreshold = 0.98, bool debug = false);

    //LDAofPCA(const char *path);

    void learn(const std::vector<Vector> &vectors, const std::vector<int> &classMembership, double pcaSelectionThreshold = 0.98, bool debug = false);

    Vector project(const Vector &vector) const;

    Vector normalizeParams(const Vector &params) { return params; }
};

}
}
