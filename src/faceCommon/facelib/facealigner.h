#ifndef FACEALIGNER_H
#define FACEALIGNER_H

#include "facelib/mesh.h"

class FaceAligner
{
public:
    FaceAligner();

    void align(Mesh &face);
};

#endif // FACEALIGNER_H
