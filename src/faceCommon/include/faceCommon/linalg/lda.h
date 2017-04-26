#pragma once

#include <opencv/cv.h>

#include "common.h"
#include "projectionbase.h"

namespace Face {
namespace LinAlg {

class LDA : public ProjectionBase
{
public:
    Matrix Wt;
    Matrix mean;

    LDA();

    LDA(const std::vector<Vector> &vectors, const std::vector<int> &classMembership, bool debug = false);

    LDA(const std::string &path);

    void learn(const std::vector<Vector> &vectors, const std::vector<int> &classMembership, bool debug = false);

    Vector project(const Vector &vector) const;

    void serialize(const std::string &path) const;

    Vector normalizeParams(const Vector &params) { return params; }
};

}
}
