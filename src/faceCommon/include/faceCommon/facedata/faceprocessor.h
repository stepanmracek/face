#pragma once

#include "facealigner.h"
#include "mesh.h"

namespace Face
{
namespace FaceData
{

class FACECOMMON_EXPORTS FaceProcessor
{
public:
    typedef cv::Ptr<FaceProcessor> Ptr;

    FaceProcessor(FaceAlignerLandmark::Ptr aligner, float smoothCoef = 0.01, int smoothIters = 10);

    FaceAlignerLandmark::Ptr aligner;
    float smoothCoef;
    int smoothIters;

    void process(Mesh &mesh, Landmarks &lm);
};

}
}
