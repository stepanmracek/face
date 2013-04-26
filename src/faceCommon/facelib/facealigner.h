#ifndef FACEALIGNER_H
#define FACEALIGNER_H

#include "facelib/mesh.h"
#include "facelib/landmarks.h"

class FaceAligner
{
private:
    int sampleStartX;
    int sampleEndX;
    int sampleStartY;
    int sampleEndY;
    int sampleStep;
    void init();

public:
    Mesh meanFace;

    FaceAligner(Mesh &meanFace);
    FaceAligner(const QString &dirWithLandmarksAndXYZfiles);

    Landmarks align(Mesh &face, int iterations = 3);
};

#endif // FACEALIGNER_H
