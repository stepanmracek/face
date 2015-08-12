#ifndef DS325SENSORFACTORY_H
#define DS325SENSORFACTORY_H

#include <Poco/Thread.h>
#include <Poco/SharedPtr.h>
#include <DepthSense.hxx>

#include "faceSensors/isensor.h"
#include "softkinetic/settings.h"
#include "softKinetic.h"

namespace Face {
namespace Sensors {
namespace SoftKinetic {

class SOFTKINETIC_EXPORTS IDS325Sensor : public Face::Sensors::ISensor
{
public:
    typedef cv::Ptr<IDS325Sensor> Ptr;

protected:
    Face::FaceData::Mesh faceMesh;
    Face::FaceData::Landmarks landmarks;

public:
    virtual ~IDS325Sensor(){}
    SensorData sensorData();
};

class SOFTKINETIC_EXPORTS DS325SensorFactory
{
private:
    DepthSense::Context context;

public:
    DS325SensorFactory();
    ~DS325SensorFactory();

    std::vector<DepthSense::Device> getConnectedDevices();

    IDS325Sensor::Ptr create(int sensorCount, const std::string &haarFaceDetectionPath,
                             const std::string &landmarkDetectorPath,
                             const Settings &settings = Settings());

    IDS325Sensor::Ptr create(const std::string &haarFaceDetectionPath,
                             const std::string &landmarkDetectorPath,
                             const Settings &settings = Settings());
};

}
}
}

#endif // IDS325SENSOR_H
