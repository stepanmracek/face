#ifndef DS325SENSORFACTORY_H
#define DS325SENSORFACTORY_H

#include <Poco/Thread.h>
#include <Poco/SharedPtr.h>
#include <DepthSense.hxx>

#include "faceSensors/isensor.h"
#include "softkinetic/settings.h"

namespace Face {
namespace Sensors {
namespace SoftKinetic {

class IDS325Sensor : public Face::Sensors::ISensor
{
public:
    typedef cv::Ptr<IDS325Sensor> Ptr;

    enum State { Off, Waiting, Positioning, Capturing, CapturingDone };

    enum PositioningOutput {
        NoData, MoveCloser, MoveFar, DontMove, LookUp, LookDown
    };

    struct Output
    {
        State state;
        State previousState;
        PositioningOutput positioningOutput;
        cv::Mat_<uchar> currentFrame;
        int capturingProgress;
        int positioningProgress;
        cv::Rect faceRegion;
        bool stopGesture;
    };

protected:
    Face::FaceData::Mesh faceMesh;
    Output output;

public:
    virtual ~IDS325Sensor(){}
    virtual Face::FaceData::Mesh &mesh() { return faceMesh; }
    const Output &getOutput() { return output; }
    virtual void doLoop() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
};

class DS325SensorFactory
{
private:
    DepthSense::Context context;

public:
    DS325SensorFactory();
    ~DS325SensorFactory();

    std::vector<DepthSense::Device> getConnectedDevices();

    IDS325Sensor::Ptr create(int sensorCount, const std::string &haarFaceDetectionPath,
                                    const Settings &settings = Settings());

    IDS325Sensor::Ptr create(const std::string &haarFaceDetectionPath, const Settings &settings = Settings());
};

}
}
}

#endif // IDS325SENSOR_H
