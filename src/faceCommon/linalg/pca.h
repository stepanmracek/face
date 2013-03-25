#ifndef PCA_H
#define PCA_H

#include <QVector>
#include <QString>

#include <opencv/cv.h>

#include "linalg/vector.h"
#include "common.h"
#include "projectionbase.h"

class PCA : public ProjectionBase
{
public:
    cv::PCA cvPca;

    PCA() {}

    PCA(QVector<Vector> &vectors, int maxComponents = 0, bool debug = false);

    PCA(const QString &path);

    int getModes() { return cvPca.eigenvalues.rows; }

    void setModes(int modes)
    {
        cvPca.eigenvectors = cvPca.eigenvectors.rowRange(0, modes);
        cvPca.eigenvalues = cvPca.eigenvalues.rowRange(0, modes);
    }

    void modesSelectionThreshold(double t = 0.98);

    double getVariation(int mode);

    void learn(QVector<Vector> &vectors, int maxComponents = 0, bool debug = false);

    void serialize(const QString &path);

    Vector project(const Vector &vector);

    Vector scaledProject(const Vector &vector);

    Vector backProject(const Vector &vector);

    Vector getMean();

    Vector normalizeParams(const Vector &params);

    Vector normalizeParams(const Vector &params, double stdMultiplier);
};

#endif // PCA_H
