#include "facealigner.h"

#include <QDir>
#include <QFileInfoList>
#include <QFileInfo>

#include "facelib/landmarkdetector.h"
#include "linalg/procrustes.h"
#include "linalg/kernelgenerator.h"

void FaceAligner::init()
{
    sampleStartX = -40;
    sampleEndX = 40;
    sampleStartY = -40;
    sampleEndY = 60;
    sampleStep = 5;
}

FaceAligner::FaceAligner(Mesh &meanFace) : meanFace(meanFace)
{
    init();
}

FaceAligner::FaceAligner(const QString &dirWithLandmarksAndXYZfiles)
{
    init();

    QDir dir(dirWithLandmarksAndXYZfiles);
    QStringList xmlFilter; xmlFilter << "*.xml";
    QFileInfoList lmFiles = dir.entryInfoList(xmlFilter, QDir::Files, QDir::Name);
    QVector<Landmarks> vecOfLandmarks;

    int lmCount = 9;
    VectorOfPoints meanLandmarks(lmCount);
    QVector<Mesh> vectorOfFaces;
    foreach (const QFileInfo &lmInfo, lmFiles)
    {
        //if (vecOfLandmarks.count() == 5) break; // DEBUG ONLY!!!

        qDebug() << lmInfo.fileName();
        Landmarks lm(lmInfo.absoluteFilePath());
        vecOfLandmarks << lm;
        cv::Point3d shift = -lm.get(Landmarks::Nosetip);
        Procrustes3D::translate(lm.points, shift);

        for (int i = 0; i < lmCount; i++)
        {
            meanLandmarks[i] += lm.points[i];
        }

        Mesh f = Mesh::fromXYZ(dirWithLandmarksAndXYZfiles + QDir::separator() + lmInfo.baseName() + ".abs.xyz");
        f.translate(shift);
        vectorOfFaces << f;
    }
    for (int i = 0; i < lmCount; i++)
    {
        meanLandmarks[i].x = meanLandmarks[i].x/lmFiles.count();
        meanLandmarks[i].y = meanLandmarks[i].y/lmFiles.count();
        meanLandmarks[i].z = meanLandmarks[i].z/lmFiles.count();
    }

    Map meanDepth = Map(320, 480);
    meanDepth.setAll(0);
    MapConverter converter;
    for (int i = 0; i < vecOfLandmarks.count(); i++)
    {
        Matrix rotation = Procrustes3D::getOptimalRotation(vecOfLandmarks[i].points, meanLandmarks);
        vectorOfFaces[i].transform(rotation);
        Map depth = SurfaceProcessor::depthmap(vectorOfFaces[i], converter, cv::Point2d(-160, -240), cv::Point2d(160, 240), 1.0, ZCoord);
        meanDepth.add(depth);
    }

    for (int y = sampleStartY; y <= sampleEndY; y += sampleStep)
    {
        for (int x = sampleStartX; x <= sampleEndX; x += sampleStep)
        {
            cv::Point2d mapPoint = converter.MeshToMapCoords(meanDepth, cv::Point2d(x, y));
            cv::Point3d meshPoint = converter.MapToMeshCoords(meanDepth, mapPoint);
            meanFace.points << meshPoint;
        }
    }
    meanFace.scale(cv::Point3d(1.0, 1.0, 1.0/vecOfLandmarks.count()));
    meanFace.recalculateMinMax();
    meanFace.calculateTriangles();
}

Procrustes3DResult FaceAligner::icpAlign(Mesh &face, int maxIterations)
{
    assert(maxIterations >= 1);
    LandmarkDetector lmDetector(face);
    Landmarks lm = lmDetector.detect();
    face.translate(-lm.get(Landmarks::Nosetip));

    Procrustes3DResult result;
    for (int iteration = 0; iteration < maxIterations; iteration++)
    {
        // Find correspondence
        VectorOfPoints referencePoints = meanFace.points;
        VectorOfPoints pointsToTransform = face.getNearestPoints(referencePoints);

        // translation
        cv::Point3d centralizeReferences = Procrustes3D::centralizedTranslation(referencePoints);
        Procrustes3D::translate(referencePoints, centralizeReferences);
        cv::Point3d centralizePointsToTransform = Procrustes3D::centralizedTranslation(pointsToTransform);
        Procrustes3D::translate(pointsToTransform, centralizePointsToTransform);

        result.preTranslations << centralizePointsToTransform;

        // SVD rotation
        Matrix rotation = Procrustes3D::getOptimalRotation(pointsToTransform, referencePoints);
        Procrustes3D::transform(pointsToTransform, rotation);
        face.transform(rotation);

        result.rotations << rotation;
        result.scaleParams << cv::Point3d(1,1,1);

        Procrustes3D::translate(referencePoints, -centralizeReferences);
        Procrustes3D::translate(pointsToTransform, -centralizeReferences);

        result.postTranslations << -centralizeReferences;

        double d = Procrustes3D::diff(pointsToTransform, referencePoints) / referencePoints.count();
        qDebug() << "FaceAligner::icpAlign" << (iteration+1) << d;
    }
}

/*Procrustes3DResult FaceAligner::icpAlignRotAndScale(Mesh &face, int maxIterations, int rotationAfter)
{
    assert(maxIterations >= 1);
    LandmarkDetector lmDetector(face);
    Landmarks lm = lmDetector.detect();
    face.translate(-lm.get(Landmarks::Nosetip));

    Procrustes3DResult result;
    MapConverter c;
    for (int iteration = 0; iteration < maxIterations; iteration++)
    {
        //cv::imshow("reference", SurfaceProcessor::depthmap(meanFace, c, 1, ZCoord).toMatrix());
        //cv::imshow("input", SurfaceProcessor::depthmap(face, c, 1, ZCoord).toMatrix());
        //cv::waitKey(0);

        // Find correspondence
        VectorOfPoints referencePoints = meanFace.points;
        VectorOfPoints pointsToTransform = face.getNearestPoints(referencePoints);

        // translation
        cv::Point3d centralizeReferences = Procrustes3D::centralizedTranslation(referencePoints);
        Procrustes3D::translate(referencePoints, centralizeReferences);
        cv::Point3d centralizePointsToTransform = Procrustes3D::centralizedTranslation(pointsToTransform);
        Procrustes3D::translate(pointsToTransform, centralizePointsToTransform);

        // SVD rotation
        Matrix rotation = Procrustes3D::getOptimalRotation(pointsToTransform, referencePoints);
        Procrustes3D::transform(pointsToTransform, rotation);
        face.transform(rotation);

        // Scale
        if (iteration > rotationAfter)
        {
            cv::Point3d scale = Procrustes3D::getOptimalScale(pointsToTransform, referencePoints);
            Procrustes3D::scale(pointsToTransform, scale);
            face.scale(scale);
        }

        Procrustes3D::translate(referencePoints, -centralizeReferences);
        Procrustes3D::translate(pointsToTransform, -centralizeReferences);

        double d = Procrustes3D::diff(pointsToTransform, referencePoints) / referencePoints.count();
        qDebug() << "FaceAligner::icpAlign" << (iteration+1) << d;
    }

    return result;
}*/
