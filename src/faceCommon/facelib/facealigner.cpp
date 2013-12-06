#include "facealigner.h"

#include <QDir>
#include <QFileInfoList>
#include <QFileInfo>

#include "facelib/landmarkdetector.h"
#include "linalg/procrustes.h"
#include "linalg/kernelgenerator.h"


FaceAligner::FaceAligner(const Mesh &referenceFace) : referenceFace(referenceFace)
{

}

void FaceAligner::alignCentralize(Mesh &face)
{
    face.centralize();
}

void FaceAligner::alignMaxZ(Mesh &face)
{
    double maxZ = -1e300;
    double index = 0;
    face.centralize();
    for (int r = 0; r < face.pointsMat.rows; r++)
    {
        double z = face.pointsMat(r, 2);
        if (z > maxZ)
        {
            index = 0;
            maxZ = z;
        }
    }

    cv::Point3d translate(face.pointsMat(index, 0), face.pointsMat(index, 1), face.pointsMat(index, 2));
    face.translate(-translate);
}

void FaceAligner::AlignNoseTip(Mesh &face)
{
    LandmarkDetector lmDetector(face);
    Landmarks lm = lmDetector.detect();
    if (lm.is(Landmarks::Nosetip))
    {
        face.translate(-lm.get(Landmarks::Nosetip));
    }
    else
    {
        alignMaxZ(face);
    }
}

VectorOfPoints getPointCloudFromMatrix(const Matrix &points)
{
    VectorOfPoints result;
    for (int r = 0; r < points.rows; r++)
    {
        result << cv::Point3d(points(r,0), points(r,1), points(r,2));
    }
    return result;
}

void FaceAligner::icpAlign(Mesh &face, int maxIterations, PreAlignTransform preAlign)
{
    qDebug() << "icpAlign()";
    Procrustes3DResult progress;

    switch (preAlign)
    {
    case Centralize:
        alignCentralize(face);
        break;
    case MaxZ:
        alignMaxZ(face);
        break;
    case NoseTipDetection:
        AlignNoseTip(face);
        break;
    }

    cv::flann::Index index;
    cv::Mat features;
    face.trainPointIndex(index, features, cv::flann::KMeansIndexParams());

    Matrix referencePoints = Matrix(referenceFace.pointsMat.rows, referenceFace.pointsMat.cols);
    Matrix transformedReference = Matrix(referenceFace.pointsMat.rows, referenceFace.pointsMat.cols);
    Matrix pointsToTransform = Matrix(referenceFace.pointsMat.rows, referenceFace.pointsMat.cols);
    for (int iteration = 0; iteration < maxIterations; iteration++)
    {
        referenceFace.pointsMat.copyTo(referencePoints);
        referenceFace.pointsMat.copyTo(transformedReference);
        for (int i = progress.preTranslations.count() - 1; i >= 0; i--)
        {
            Procrustes3D::translate(transformedReference, -progress.postTranslations[i]);
            Procrustes3D::inverseTransform(transformedReference, progress.rotations[i]);
            Procrustes3D::translate(transformedReference, -progress.preTranslations[i]);
        }

        face.getNearestPoints(transformedReference, index, pointsToTransform);

        // translation
        cv::Point3d centralizeReferences = Procrustes3D::centralizedTranslation(referencePoints);
        Procrustes3D::translate(referencePoints, centralizeReferences);

        cv::Point3d centralizePointsToTransform = Procrustes3D::centralizedTranslation(pointsToTransform);
        Procrustes3D::translate(pointsToTransform, centralizePointsToTransform);
        face.translate(centralizePointsToTransform);
        progress.preTranslations << centralizePointsToTransform;

        // SVD rotation
        Matrix rotation = Procrustes3D::getOptimalRotation(pointsToTransform, referencePoints);
        face.transform(rotation);
        progress.rotations << rotation;

        // post translation
        face.translate(-centralizeReferences);
        progress.postTranslations << -centralizeReferences;
    }

    qDebug() << "done";
}
