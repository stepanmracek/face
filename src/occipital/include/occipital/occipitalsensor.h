#ifndef OCCIPITALSENSOR_H
#define OCCIPITALSENSOR_H

#include "faceSensors/isensor.h"
#include "OpenNI.h"

namespace Face
{
namespace Sensors
{
namespace Occipital
{

class OccipitalSensor : public Face::Sensors::ISensor, public openni::VideoStream::NewFrameListener
{
protected:
    openni::Device device;
    openni::VideoStream irStream;
    openni::VideoFrameRef irFrame;
    Matrix irData;

    openni::VideoStream depthStream;
    openni::VideoFrameRef depthFrame;
    Matrix depthData;

    static const int Width = 320;
    static const int Height = 240;

    Face::FaceData::Mesh m;

    void configureDepth();
    void configureIR();
    void onNewFrame(openni::VideoStream& stream);

public:
    OccipitalSensor();
    ~OccipitalSensor();

    void scan();
    Face::FaceData::Mesh &mesh() { return m; }
};

}
}
}

#endif // OCCIPITALSENSOR_H
