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

    PCA(QVector<Matrix> &vectors, int maxComponents = 0, bool debug = false);

    PCA(const char *path);

    int getModes() { return cvPca.eigenvalues.rows; }

    void setModes(int modes)
    {
        cvPca.eigenvectors = cvPca.eigenvectors.rowRange(0, modes);
        cvPca.eigenvalues = cvPca.eigenvalues.rowRange(0, modes);
    }

    void modesSelectionThreshold(double t = 0.98);

    void learn(QVector<Matrix> &vectors, int maxComponents = 0, bool debug = false);

    void serialize(const char *path);

    Matrix project(const Matrix &vector);

    QVector<Matrix> project(const QVector<Matrix> &vectors);

    Matrix normalizedProject(const Matrix &vector);

    Matrix backProject(const Matrix &vector);
};

#endif // PCA_H
