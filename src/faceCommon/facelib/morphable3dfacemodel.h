#ifndef MORPHABLE3DFACEMODEL_H
#define MORPHABLE3DFACEMODEL_H

#include <QVector>

#include "linalg/pca.h"
#include "facelib/mesh.h"
#include "linalg/vector.h"
#include "facelib/map.h"
#include "facelib/maskedvector.h"
#include "facelib/landmarks.h"
#include "linalg/procrustes.h"

class Morphable3DFaceModel
{
public:
    PCA pcaForZcoord;
    PCA pcaForTexture;
    Mesh mesh;
    Landmarks landmarks;
    Matrix mask;
    //Map depthMapMask;

    Morphable3DFaceModel(const QString &pcaPathForZcoord, const QString &pcaPathForTexture,
                         const QString &maskPath, const QString &landmarksPath, int width);

    void setModelParams(Vector &zcoordParams);
    void setModelParams(Vector &zcoordParams, Vector &textureParams);

    Procrustes3DResult align(Mesh &inputMesh, Landmarks &inputLandmarks, int iterations);

    void morphModel(Mesh &alignedMesh);


    static void align(QVector<Mesh> &meshes,
                      QVector<VectorOfPoints> &controlPoints,
                      int iterations);

    static void create(QVector<Mesh> &meshes,
                       QVector<VectorOfPoints> &controlPoints,
                       int iterations,
                       const QString &pcaForZcoordFile,
                       const QString &pcaForTextureFile,
                       const QString &flagsFile,
                       const QString &meanControlPointsFile,
                       Map &mapMask);
};

#endif // MORPHABLE3DFACEMODEL_H
