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
    for (int y = -10; y <= 20; y += 5)
    {
        for (int x = -10; x <= 10; x += 5)
        {
            cv::Point2d mapPoint = converter.MeshToMapCoords(depth, cv::Point2d(x, y));
            cv::Point3d meshPoint = converter.MapToMeshCoords(depth, mapPoint);
            pointsToAlign << meshPoint;
        }
    }
    Procrustes3D::getOptimalRotation(pointsToAlign, pointsOnMeanFace);

}
