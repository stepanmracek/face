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

                cv::circle(img, mapPoint, 2, 0);
            }
        }

        cv::imshow("sampled points", img);
        cv::waitKey(500);
    }

    //Procrustes3D::getOptimalRotation(pointsToAlign, pointsOnMeanFace);
}
