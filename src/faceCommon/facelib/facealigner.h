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
    Mesh referenceFace;

    FaceAligner(Mesh &referenceFace);
    FaceAligner(const QString &dirWithLandmarksAndXYZfiles);

    Procrustes3DResult icpAlign(Mesh &face, int maxIterations);
    //Procrustes3DResult icpAlignRotAndScale(Mesh &face, int maxIterations, int rotationAfter);
};

#endif // FACEALIGNER_H
