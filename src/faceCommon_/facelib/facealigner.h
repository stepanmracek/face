#ifndef FACEALIGNER_H
#define FACEALIGNER_H

#include "facelib/mesh.h"
#include "facelib/landmarks.h"
#include "linalg/procrustes.h"

class FaceAligner
{
private:
    void AlignNoseTip(Mesh &face);
    void alignMaxZ(Mesh &face);
    void alignCentralize(Mesh &face);

public:
    enum PreAlignTransform { None, NoseTipDetection, MaxZ, Centralize };

    const Mesh referenceFace;

    FaceAligner() {}
    FaceAligner(const Mesh &referenceFace);
    void icpAlign(Mesh &face, int maxIterations, PreAlignTransform preAlign);
};

#endif // FACEALIGNER_H
