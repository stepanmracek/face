#include "facealigner.h"

#include "facelib/landmarkdetector.h"
#include "linalg/procrustes.h"

FaceAligner::FaceAligner()
{
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
    for (int y = -20; y <= 40; y += 10)
    {
        for (int x = -20; x <= 20; x += 10)
        {
            cv::Point2d mapPoint = converter.MeshToMapCoords(depth, cv::Point2d(x, y));
            cv::Point3d meshPoint = converter.MapToMeshCoords(depth, mapPoint);
            pointsToAlign << meshPoint;

            cv::circle(depthMatrix, mapPoint, 2, 0);
        }
    }

    cv::imshow("sampled points", depthMatrix);
    cv::waitKey();
    //Procrustes3D::getOptimalRotation(pointsToAlign, pointsOnMeanFace);
}
