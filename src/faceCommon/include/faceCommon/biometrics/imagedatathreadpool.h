#ifndef IMAGEDATATHREADPOOL_H
#define IMAGEDATATHREADPOOL_H

#include <Poco/ThreadPool.h>

#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceCommon/biometrics/multiextractor.h"

namespace Face
{
namespace Biometrics
{

class ImageDataThreadPool : public Poco::ThreadPool
{
    class TextureThread : public Poco::Runnable
    {
        const Face::FaceData::Mesh *mesh;

    public:
        Matrix result;

        void setUp(const Face::FaceData::Mesh *mesh);
        void run();
    };

    class DepthAndCurvatureThread : public Poco::Runnable
    {
        const Face::FaceData::Mesh *mesh;

    public:
        Matrix depth;
        Matrix mean;
        Matrix gauss;
        Matrix index;
        Matrix eigencur;

        void setUp(const Face::FaceData::Mesh *mesh);
        void run();
    };

    class LargeDepthThread : public Poco::Runnable
    {
        const Face::FaceData::Mesh *mesh;

    public:
        Face::FaceData::Map result;
        Face::FaceData::MapConverter converter;

        void setUp(const Face::FaceData::Mesh *mesh);
        void run();
    };

    TextureThread textureThread;
    DepthAndCurvatureThread depthAndCurvatureThread;
    LargeDepthThread largeDepthThread;

public:

    MultiExtractor::ImageData process(const FaceData::Mesh *mesh);

};

}
}

#endif // IMAGEDATATHREADPOOL_H
