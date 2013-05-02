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
        f.move(shift);
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

Landmarks FaceAligner::align(Mesh &face, int maxIterations)
{
    assert(maxIterations >= 1);
    LandmarkDetector lmDetector(face);
    Landmarks lm = lmDetector.detect();

    Matrix smoothKernel = KernelGenerator::gaussianKernel(5);
    double totalMinD = 1e300;
    for (int iteration = 0; iteration < maxIterations; iteration ++)
    {
        qDebug() << "FaceAligner::align" << (iteration+1) << "/" << maxIterations;

        MapConverter converter;
        Map depth = SurfaceProcessor::depthmap(face, converter, 1.0, ZCoord);
        cv::imshow("smooth1", depth.toMatrix());
        depth.applyFilter(smoothKernel, 3, true);
        //cv::imshow("smooth2", depth.toMatrix());
        cv::waitKey();

        double minTheta;
        double minD = 1e300;
        Matrix minRotation;
        cv::Point3d minMove;
        bool improve = false;

        for (double theta = -0.15; theta <= 0.15; theta += 0.005)
        {
            double cosT = cos(theta);
            double sinT = sin(theta);

            VectorOfPoints pointsToTransform;
            VectorOfPoints referencePoints;

            int index = 0;
            for (int y = sampleStartY; y <= sampleEndY; y += sampleStep)
            {
                for (int x = sampleStartX; x <= sampleEndX; x += sampleStep)
                {
                    double xr = x * cosT - y * sinT;
                    double yr = x * sinT + y * cosT;

                    cv::Point2d mapPoint = converter.MeshToMapCoords(depth, cv::Point2d(xr, yr));
                    bool success;
                    cv::Point3d meshPoint = converter.MapToMeshCoords(depth, mapPoint, &success);
                    if (success)
                    {
                        pointsToTransform << meshPoint;
                        referencePoints << meanFace.points[index];
                    }
                    index++;
                }
            }

            // theta rotation
            Procrustes3D::rotate(pointsToTransform, 0, 0, -theta);

            // translation
            cv::Point3d move = Procrustes3D::getOptimalTranslation(pointsToTransform, referencePoints);
            Procrustes3D::translate(pointsToTransform, move);

            // SVD rotation
            Matrix rotation = Procrustes3D::getOptimalRotation(pointsToTransform, referencePoints);
            Procrustes3D::transform(pointsToTransform, rotation);

            double d = Procrustes3D::diff(pointsToTransform, referencePoints);

            if (d < minD && d < totalMinD)
            {
                improve = true;
                minD = d;
                totalMinD = d;
                minTheta = theta;
                minRotation = rotation.clone();
                minMove = move;
            }
        }

        qDebug() << minTheta << minD << improve;

        if (improve)
        {
            face.rotate(0, 0, -minTheta);
            face.move(minMove);
            face.transform(minRotation);

            Procrustes3D::rotate(lm.points, 0, 0, -minTheta);
            Procrustes3D::translate(lm.points, minMove);
            Procrustes3D::transform(lm.points, minRotation);
        }
        else
        {
            break;
        }
    }

    return lm;
}
