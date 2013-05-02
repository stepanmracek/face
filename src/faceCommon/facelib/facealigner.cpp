#include "facealigner.h"

#include <QDir>
#include <QFileInfoList>
#include <QFileInfo>

#include "facelib/landmarkdetector.h"
#include "linalg/procrustes.h"

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

        Mesh f = Mesh::fromXYZFile(dirWithLandmarksAndXYZfiles + QDir::separator() + lmInfo.baseName() + ".abs.xyz");
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

Landmarks FaceAligner::align(Mesh &face, int iterations)
{
    assert(iterations >= 1);
    LandmarkDetector lmDetector(face);
    Landmarks lm = lmDetector.detect();

    for (int iteration = 0; iteration < iterations; iteration ++)
    {
        qDebug() << "FaceAligner::align" << (iteration+1) << "/" << iterations;

        MapConverter converter;
        Map depth = SurfaceProcessor::depthmap(face, converter, 1.0, ZCoord);

        double minTheta;
        double minD = 1e300;
        Matrix minRotation;
        cv::Point3d minMove;

        for (double theta = -0.15; theta <= 0.15; theta += 0.01)
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

            if (d < minD)
            {
                minD = d;
                minTheta = theta;
                minRotation = rotation.clone();
                minMove = move;
            }
        }

        qDebug() << minTheta;

        face.rotate(0, 0, -minTheta);
        face.move(minMove);
        face.transform(minRotation);

        Procrustes3D::rotate(lm.points, 0, 0, -minTheta);
        Procrustes3D::translate(lm.points, minMove);
        Procrustes3D::transform(lm.points, minRotation);
    }

    /*double minTheta;
    double minD = 1e300;
    Matrix rotation;

    for (double theta = -0.15; theta <= 0.15; theta += 0.01)
    {
        Mesh faceCopy(face);
        faceCopy.rotate(0, 0, theta);
        MapConverter converter;
        Map depth = SurfaceProcessor::depthmap(faceCopy, converter, 1.0, ZCoord);
        //Matrix testDepth = depth.toMatrix();

        Mesh sampledFace;
        VectorOfPoints referencePoints;

        int index = 0;
        for (int y = sampleStartY; y <= sampleEndY; y += sampleStep)
        {
            for (int x = sampleStartX; x <= sampleEndX; x += sampleStep)
            {
                cv::Point2d mapPoint = converter.MeshToMapCoords(depth, cv::Point2d(x, y));
                bool success;
                cv::Point3d meshPoint = converter.MapToMeshCoords(depth, mapPoint, &success);
                if (success)
                {
                    sampledFace.points << meshPoint;
                    referencePoints << meanFace.points[index];
                    //cv::circle(testDepth, mapPoint, 2, 0);
                }
                index++;
            }
        }

        //sampledFace.recalculateMinMax();
        //sampledFace.calculateTriangles();
        //MapConverter testMC;
        //Map testMap = SurfaceProcessor::depthmap(sampledFace, testMC, 1, ZCoord);
        //cv::imshow("test", testMap.toMatrix());
        //cv::imshow("testDepth", testDepth);
        //cv::waitKey(1);

        //Matrix rotationCandidate = Procrustes3D::getOptimalRotation(sampledFace.points, referencePoints);
        //Procrustes3D::transform(sampledFace.points, rotationCandidate);
        double d = Procrustes3D::diff(sampledFace.points, referencePoints);

        if (d < minD)
        {
            minD = d;
            minTheta = theta;
            //rotation = rotationCandidate.clone();
        }

        qDebug() << theta << d << minD;
    }

    qDebug() << "theta" << minTheta;
    face.rotate(0, 0, minTheta);
    //face.transform(rotation);*/

    return lm;
}
