#ifndef PCA_H
#define PCA_H

#include <opencv/cv.h>

#include "vector.h"
#include "common.h"
#include "projectionbase.h"
#include "iserializable.h"

namespace Face {
namespace LinAlg {

class PCA : public ProjectionBase, public ISerializable
{
public:
    cv::PCA cvPca;

    PCA() {}

    PCA(const std::vector<Vector> &vectors, int maxComponents = 0, bool debug = false);

    PCA(const std::string &path);

    int getModes() const { return cvPca.eigenvalues.rows; }

    void setModes(int modes)
    {
        cvPca.eigenvectors = cvPca.eigenvectors.rowRange(0, modes);
        cvPca.eigenvalues = cvPca.eigenvalues.rowRange(0, modes);
    }

    void modesSelectionThreshold(double t = 0.98);

    double getVariation(int mode);

    void learn(const std::vector<Vector> &vectors, int maxComponents = 0, bool debug = false);

    void serialize(cv::FileStorage &storage) const;

    void deserialize(cv::FileStorage &storage);

    Vector project(const Vector &vector) const;

    Vector scaledProject(const Vector &vector) const;

    Vector backProject(const Vector &vector) const;

    Vector getMean() const;

    Vector normalizeParams(const Vector &params);

    Vector normalizeParams(const Vector &params, double stdMultiplier);
};

}
}

#endif // PCA_H
