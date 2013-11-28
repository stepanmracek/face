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

FaceAligner::FaceAligner(const Mesh &referenceFace) : referenceFace(referenceFace)
{
    init();
}

/*FaceAligner::FaceAligner(const QString &dirWithLandmarksAndXYZfiles)
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
            referenceFace.points << meshPoint;
        }
    }
    referenceFace.scale(cv::Point3d(1.0, 1.0, 1.0/vecOfLandmarks.count()));
    referenceFace.recalculateMinMax();
    referenceFace.calculateTriangles();
}*/

Procrustes3DResult FaceAligner::icpAlignDeprecated(Mesh &face, int maxIterations)
{
    assert(maxIterations >= 1);
    Procrustes3DResult result;
    LandmarkDetector lmDetector(face);
    Landmarks lm = lmDetector.detect();
    if (!lm.is(Landmarks::Nosetip)) return result;
    face.translate(-lm.get(Landmarks::Nosetip));

    for (int iteration = 0; iteration < maxIterations; iteration++)
    {
        /*MapConverter c;
        Map depth = SurfaceProcessor::depthmap(face, c, 1, ZCoord);
        cv::imshow("depth", depth.toMatrix());
        cv::waitKey();*/

        // Find correspondence
        VectorOfPoints referencePoints = referenceFace.points;
        VectorOfPoints pointsToTransform = face.getNearestPoints(referencePoints);

        // translation
        cv::Point3d centralizeReferences = Procrustes3D::centralizedTranslation(referencePoints);
        Procrustes3D::translate(referencePoints, centralizeReferences);
        cv::Point3d centralizePointsToTransform = Procrustes3D::centralizedTranslation(pointsToTransform);
        Procrustes3D::translate(pointsToTransform, centralizePointsToTransform);

        result.preTranslations << centralizePointsToTransform;

        // SVD rotation
        Matrix rotation = Procrustes3D::getOptimalRotation(pointsToTransform, referencePoints);
        //Procrustes3D::transform(pointsToTransform, rotation);
        face.transform(rotation);

        result.rotations << rotation;
        result.scaleParams << cv::Point3d(1,1,1);

        //Procrustes3D::translate(referencePoints, -centralizeReferences);
        //Procrustes3D::translate(pointsToTransform, -centralizeReferences);

        result.postTranslations << -centralizeReferences;

        //double d = Procrustes3D::diff(pointsToTransform, referencePoints) / referencePoints.count();
        //qDebug() << "FaceAligner::icpAlign" << (iteration+1) << d;
    }
    return result;
}

void FaceAligner::alignCentralize(Mesh &face)
{
    face.centralize();
}

void FaceAligner::alignMaxZ(Mesh &face)
{
    double maxZ = -1e300;
    face.centralize();
    cv::Point3d translate;
    foreach (const cv::Point3d &p, face.points)
    {
        if (p.z > maxZ)
        {
            translate = -p;
            maxZ = p.z;
        }
    }
    face.translate(translate);
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
    for (int iteration = 0; iteration < maxIterations; iteration++)
    {
        // Find correspondence
        VectorOfPoints referencePoints = referenceFace.points;
        VectorOfPoints transformedReference = referencePoints;
        for (int i = progress.preTranslations.count() - 1; i >= 0; i--)
        {
            Procrustes3D::translate(transformedReference, -progress.postTranslations[i]);
            Matrix inv = progress.rotations[i].inv();
            Procrustes3D::transform(transformedReference, inv);
            Procrustes3D::translate(transformedReference, -progress.preTranslations[i]);
        }

        VectorOfPoints pointsToTransform = face.getNearestPoints(transformedReference, index);

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
}
