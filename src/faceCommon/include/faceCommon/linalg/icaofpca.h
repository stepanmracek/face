#pragma once

#include "ica.h"
#include "pca.h"
#include "vector.h"
#include "projectionbase.h"

namespace Face {
namespace LinAlg {

class FACECOMMON_EXPORTS ICAofPCA : public ProjectionBase
{
public:
    ICA ica;
    PCA pca;

    ICAofPCA() {}

    ICAofPCA(const std::vector<Vector> &vectors,
             double pcaSelectionThreshold = 0.98,
             int independentComponentCount = 0,
             double epsICA = 1e-10,
             bool debug = false);


    void learn(const std::vector<Vector> &vectors,
               double pcaSelectionThreshold = 0.98,
               int independentComponentCount = 0,
               double epsICA = 1e-10,
               bool debug = false);

    Vector project(const Vector &vector) const;

    Vector whiten(const Vector &vector) const;

    std::vector<Vector> whiten(const std::vector<Vector> &vectors) const;

    Vector backProject(const Vector &vector);

    Vector normalizeParams(const Vector &params) { return params; }
};

}
}
