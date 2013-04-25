#include "facealigner.h"

#include <QDir>
#include <QFileInfoList>
#include <QFileInfo>

#include "facelib/landmarkdetector.h"
#include "linalg/procrustes.h"

FaceAligner::FaceAligner(Mesh &meanFace) : meanFace(meanFace)
{

}

FaceAligner::FaceAligner(const QString &dirWithLandmarksAndXYZfiles)
{
    QDir dir(dirWithLandmarksAndXYZfiles);
    QStringList xmlFilter; xmlFilter << "*.xml";
    QFileInfoList lmFiles = dir.entryInfoList(xmlFilter, QDir::Files, QDir::Name);
    foreach (const QFileInfo &lmInfo, lmFiles)
    {
        qDebug() << lmInfo.fileName() << lmInfo.baseName() + ".abs.xyz";
    }
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
        for (int y = -40; y <= 60; y += 5)
        {
            for (int x = -40; x <= 40; x += 5)
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
