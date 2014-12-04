#ifndef FACEPROCESSOR_H
#define FACEPROCESSOR_H

#include "facealigner.h"
#include "mesh.h"

namespace Face
{
namespace FaceData
{

class FaceProcessor
{
public:
    typedef cv::Ptr<FaceProcessor> Ptr;

    FaceProcessor(FaceAligner::Ptr aligner, float smoothCoef = 0.01, int smoothIters = 10);

    FaceAligner::Ptr aligner;
    FaceAligner::PreAlignTransform prealign;
    int icpIterations;
    float smoothCoef;
    int smoothIters;

    void process(Mesh &mesh);
};

}
}

#endif // FACEPROCESSOR_H
