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
        if (vecOfLandmarks.count() == 5) break; // DEBUG ONLY!!!

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
            //qDebug() << x << mapPoint.x << meshPoint.x;
            meanFace.points << meshPoint;
        }
    }
    meanFace.scale(cv::Point3d(1,1,1.0/vecOfLandmarks.count()));
    meanFace.recalculateMinMax();
    meanFace.calculateTriangles();
}

void FaceAligner::align(Mesh &face)
{
    LandmarkDetector lmDetector(face);
    Landmarks lm = lmDetector.Detect();
    face.move(-lm.get(Landmarks::Nosetip));

    MapConverter converter;
    Map depth = SurfaceProcessor::depthmap(face, converter, 1.0, ZCoord);

    VectorOfPoints pointsOnMeanFace; // TODO!!!
    VectorOfPoints pointsToAlign;
    Matrix depthMatrix = depth.toMatrix(); // DEBUG

    for (double theta = -0.25; theta <= 0.25; theta += 0.025)
    {
        double cosT = cos(theta);
        double sinT = sin(theta);
        Matrix img = depthMatrix.clone();
        for (int y = sampleStartY; y <= sampleEndY; y += sampleStep)
        {
            for (int x = sampleStartX; x <= sampleEndX; x += sampleStep)
            {
                double xr = x * cosT - y * sinT;
                double yr = x * sinT + y * cosT;
                cv::Point2d mapPoint = converter.MeshToMapCoords(depth, cv::Point2d(xr, yr));
                cv::Point3d meshPoint = converter.MapToMeshCoords(depth, mapPoint);
                pointsToAlign << meshPoint;

                cv::circle(img, mapPoint, 1.0, cv::Scalar(0));
            }
        }

        cv::imshow("sampled points", img);
        cv::waitKey(1000);
    }

    //Procrustes3D::getOptimalRotation(pointsToAlign, pointsOnMeanFace);
}
