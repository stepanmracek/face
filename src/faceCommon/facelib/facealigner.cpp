#include "facealigner.h"

#include "facelib/landmarkdetector.h"

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

    for (int y = -10; y <= 20; y += 5)
    {
        for (x = -10; x <= 10; x += 5)
        {
            cv::Point2d mapPoint = converter.MeshToMapCoords(depth, cv::Point2d(x, y));
        }
    }

}
