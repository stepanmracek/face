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

    void AlignNoseTip(Mesh &face);
    void alignMaxZ(Mesh &face);
    void alignCentralize(Mesh &face);

public:
    enum PreAlignTransform { None, NoseTipDetection, MaxZ, Centralize };

    const Mesh referenceFace;

    FaceAligner() {}
    FaceAligner(const Mesh &referenceFace);
    //FaceAligner(const QString &dirWithLandmarksAndXYZfiles);

    Procrustes3DResult icpAlignDeprecated(Mesh &face, int maxIterations);
    void icpAlign(Mesh &face, int maxIterations, PreAlignTransform preAlign);
    //Procrustes3DResult icpAlignRotAndScale(Mesh &face, int maxIterations, int rotationAfter);
};

#endif // FACEALIGNER_H
