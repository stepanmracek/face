#include "faceCommon/biometrics/imagedatathreadpool.h"

using namespace Face::Biometrics;

void ImageDataThreadPool::TextureThread::setUp(const FaceData::Mesh *mesh)
{
    this->mesh = mesh;
}

void ImageDataThreadPool::TextureThread::run()
{
    cv::Rect roi(25, 15, 100, 90);
    FaceData::MapConverter converter;
    FaceData::Map textureMap = FaceData::SurfaceProcessor::depthmap(*mesh, converter, cv::Point2d(-75, -75),
                                                                    cv::Point2d(75, 75), 1,
                                                                    FaceData::SurfaceProcessor::Texture_I);
    result = textureMap.toMatrix(0, 0, 255)(roi);
}

void ImageDataThreadPool::DepthAndCurvatureThread::setUp(const Face::FaceData::Mesh *mesh)
{
    this->mesh = mesh;
}

void ImageDataThreadPool::DepthAndCurvatureThread::run()
{
    cv::Rect roi(25, 15, 100, 90);
    FaceData::MapConverter converter;

    FaceData::Map depthmap = FaceData::SurfaceProcessor::depthmap(*mesh, converter, cv::Point2d(-75, -75),
                                                                  cv::Point2d(75, 75), 1,
                                                                  FaceData::SurfaceProcessor::ZCoord);
    depthmap.bandPass(-70, 10, false, false);
    depth = depthmap.toMatrix(0, -70, 10)(roi);

    FaceData::Map smoothedDepthmap = depthmap;
    smoothedDepthmap.applyCvGaussBlur(7, 3);
    FaceData::CurvatureStruct cs = FaceData::SurfaceProcessor::calculateCurvatures(smoothedDepthmap);

    cs.curvatureMean.bandPass(-0.1, 0.1, false, false);
    mean = cs.meanMatrix()(roi);

    cs.curvatureGauss.bandPass(-0.01, 0.01, false, false);
    gauss = cs.gaussMatrix()(roi);

    cs.curvatureIndex.bandPass(0, 1, false, false);
    index = cs.indexMatrix()(roi);

    cs.curvaturePcl.bandPass(0, 0.0025, false, false);
    eigencur = cs.pclMatrix()(roi);
}

void ImageDataThreadPool::LargeDepthThread::setUp(const FaceData::Mesh *mesh)
{
    this->mesh = mesh;
}

void ImageDataThreadPool::LargeDepthThread::run()
{
    result = FaceData::SurfaceProcessor::depthmap(*mesh, converter, 2.0, FaceData::SurfaceProcessor::ZCoord);
}

MultiExtractor::ImageData ImageDataThreadPool::process(const Face::FaceData::Mesh *mesh)
{
    MultiExtractor::ImageData result;

    textureThread.setUp(mesh);
    depthAndCurvatureThread.setUp(mesh);
    largeDepthThread.setUp(mesh);

    start(textureThread, "extract-texture");
    start(depthAndCurvatureThread, "extract-curvature");
    start(largeDepthThread, "extract-largeDepth");

    joinAll();

    result.depthConverter = largeDepthThread.converter;
    result.largeDepth = largeDepthThread.result;

    result.typeDict["texture"] = textureThread.result.clone();

    result.typeDict["depth"] = depthAndCurvatureThread.depth.clone();
    result.typeDict["mean"] = depthAndCurvatureThread.mean.clone();
    result.typeDict["gauss"] = depthAndCurvatureThread.gauss.clone();
    result.typeDict["index"] = depthAndCurvatureThread.index.clone();
    result.typeDict["eigencur"] = depthAndCurvatureThread.eigencur.clone();

    return result;
}
