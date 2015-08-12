#include "faceCommon/facedata/faceprocessor.h"

#include "faceCommon/facedata/surfaceprocessor.h"

using namespace Face::FaceData;

FaceProcessor::FaceProcessor(FaceAlignerLandmark::Ptr aligner, float smoothCoef, int smoothIters) :
    aligner(aligner),
    smoothCoef(smoothCoef),
    smoothIters(smoothIters)
{

}

void FaceProcessor::process(Mesh &mesh, Landmarks &landmarks)
{
    if (smoothIters > 0)
        SurfaceProcessor::mdenoising(mesh, smoothCoef, smoothIters, smoothIters);
	aligner->align(mesh, landmarks);
}
