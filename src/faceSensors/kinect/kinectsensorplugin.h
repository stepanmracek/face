#ifndef KINECTSENSORPLUGIN_H
#define KINECTSENSORPLUGIN_H

#include "facetrack/realtimetrack.h"
#include "facelib/mesh.h"
#include "facelib/facealigner.h"

class KinectSensorPlugin
{
private:
    static const int WIDTH = 640;
    static const int HEIGHT = 480;
    static const int CHANNELS = 3;
    static const int CONSECUTIVE_DETECTIONS = 10;
    static const int CONSECUTIVE_NO_DATA = 50;
    static const double DATA_RATIO = 0.5;
    static const double POSITIONING_MAX_DISTANCE = 870;
    static const double POSITIONING_MIN_DISTANCE = 830;
    static const double MAX_GET_DATA_DISTANCE = 1000;
    static const int ICP_ITERATIONS = 10;

    unsigned char *rgbBuffer;
    double *depthBuffer;
    bool *depthMask;
    RealTimeTracker detector;
    cv::RotatedRect rotRect;
    cv::Rect rect;
    int ellipsePointsCount;
    FaceAligner aligner;

    enum PositionResponse { NoData, Ok, MoveCloser, MoveFar };
    PositionResponse depthStats();
    void putText(const char *text);

    enum State { Off, Waiting, Positioning, Capturing };
    State state;
    void drawGUI();
    void getData();
    bool positionInterupted;

public:
    ImageGrayscale img;
    Mesh *mesh;

    KinectSensorPlugin(const QString &faceDetectorPath, const QString &objModelPathForAlign);
    ~KinectSensorPlugin();

    void scanFace();

    void go();
    void off() {}
    void wait();
    void position();
    void capture();
    void deleteMesh();
    void align();
    bool isPositionInterupted() { return positionInterupted; }

    bool isKinectPluggedIn();
};

#endif // KINECTSENSORPLUGIN_H
