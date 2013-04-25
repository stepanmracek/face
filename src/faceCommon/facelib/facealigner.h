#ifndef FACEALIGNER_H
#define FACEALIGNER_H

#include "facelib/mesh.h"

class FaceAligner
{
public:
    Mesh meanFace;

    FaceAligner(Mesh &meanFace);
    FaceAligner(const QString &dirWithLandmarksAndXYZfiles);

    void align(Mesh &face);
};

#endif // FACEALIGNER_H
