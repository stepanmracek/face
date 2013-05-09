#ifndef FACEALIGNER_H
#define FACEALIGNER_H

#include "facelib/mesh.h"
#include "facelib/landmarks.h"
#include "linalg/procrustes.h"

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

    void icpAlign(Mesh &face, int maxIterations);
    Procrustes3DResult icpAlignRotAndScale(Mesh &face, int maxIterations);
};

#endif // FACEALIGNER_H
