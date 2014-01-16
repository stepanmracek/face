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
    Face::LinAlg::PCA pcaForZcoord;
    Face::LinAlg::PCA pcaForTexture;
    Face::LinAlg::PCA pca;

    Face::FaceData::Mesh mesh;
    Face::FaceData::Landmarks landmarks;
    Matrix mask;

    Morphable3DFaceModel(const QString &pcaPathForZcoord, const QString &pcaPathForTexture, const QString &pcaFile,
                         const QString &maskPath, const QString &landmarksPath, int width);

    void setModelParams(Face::LinAlg::Vector &commonParams);
    void setModelParams(Face::LinAlg::Vector &zcoordParams, Face::LinAlg::Vector &textureParams);

    Face::FaceData::Mesh morph(Face::FaceData::Mesh &inputMesh, Face::FaceData::Landmarks &inputLandmarks, int iterations);
    Face::FaceData::Mesh morph(Face::FaceData::Mesh &inputMesh, int iterations);

    static void align(QVector<Face::FaceData::Mesh> &meshes,
                      QVector<Face::FaceData::VectorOfPoints> &controlPoints,
                      int iterations, bool scale, bool centralize);

    static void create(QVector<Face::FaceData::Mesh> &meshes,
                       QVector<Face::FaceData::VectorOfPoints> &controlPoints,
                       int iterations,
                       const QString &pcaForZcoordFile,
                       const QString &pcaForTextureFile,
                       const QString &pcaFile,
                       const QString &flagsFile,
                       const QString &meanControlPointsFile,
                       Face::FaceData::Map &mapMask, bool scale, bool centralize);

private:
    Face::LinAlg::Procrustes3DResult align(Face::FaceData::Mesh &inputMesh, Face::FaceData::Landmarks &inputLandmarks,
                                           int iterations, bool scale);
    void morphModel(Face::FaceData::Mesh &alignedMesh);
};

#endif // MORPHABLE3DFACEMODEL_H
