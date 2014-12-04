#include "faceCommon/facedata/faceprocessor.h"

#include "faceCommon/facedata/surfaceprocessor.h"

using namespace Face::FaceData;

FaceProcessor::FaceProcessor(FaceAligner::Ptr aligner, float smoothCoef, int smoothIters) :
    aligner(aligner),
    icpIterations(100),
    smoothCoef(smoothCoef),
    smoothIters(smoothIters)
{

}

void FaceProcessor::process(Mesh &mesh)
{
    if (smoothIters > 0)
        SurfaceProcessor::mdenoising(mesh, smoothCoef, smoothIters, smoothIters);
    aligner->icpAlign(mesh, icpIterations, FaceAligner::CVTemplateMatching);
}
