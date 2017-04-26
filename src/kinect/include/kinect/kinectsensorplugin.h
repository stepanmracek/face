#ifndef KINECTSENSORPLUGIN_H
#define KINECTSENSORPLUGIN_H

#include "faceCommon/objectdetection/tracker.h"
#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/facealigner.h"
#include "faceSensors/isensor.h"
#include "faceCommon/settings/settings.h"

namespace Face {
namespace Sensors {
namespace Kinect {

class KinectSensorPlugin : public Face::Sensors::ISensor
{
private:
    int WIDTH;
    int HEIGHT;
    int CHANNELS;
    int CONSECUTIVE_DETECTIONS;
    int CONSECUTIVE_NO_DATA;
    double DATA_RATIO;
    double POSITIONING_MAX_DISTANCE;
    double POSITIONING_MIN_DISTANCE;
    double MAX_GET_DATA_DISTANCE;

    unsigned char *rgbBuffer;
    double *depthBuffer;
    bool *depthMask;
    Face::ObjectDetection::Tracker detector;
    cv::RotatedRect rotRect;
    cv::Rect rect;
    int ellipsePointsCount;

    enum PositionResponse { NoData, Ok, MoveCloser, MoveFar };
    PositionResponse depthStats();
    void putText(const char *text);

    enum State { Off, Waiting, Positioning, Capturing };
    State state;
    void drawGUI();
    void getData();
    bool positionInterupted;
    Face::FaceData::Mesh *_mesh;

public:
    ImageGrayscale img;

    KinectSensorPlugin(const std::string &faceDetectorPath = Face::Settings::instance().settingsMap[Face::Settings::CascadeFaceDetectorPathKey]);
    ~KinectSensorPlugin();

    void scan();
    virtual SensorData sensorData()
    {
        SensorData d;
        d.processedScan.mesh = *_mesh;
        d.rawScan.mesh = *_mesh;
        return d;
    }

    void doLoop();
    void start() {} //TODO
    void stop() {} //TODO
    void off() {}
    void wait();
    void position();
    void capture();
    void deleteMesh();
    bool isPositionInterupted() { return positionInterupted; }

    bool isKinectPluggedIn();
};

}
}
}

#endif // KINECTSENSORPLUGIN_H
