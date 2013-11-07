#ifndef KINECTSENSORPLUGIN_H
#define KINECTSENSORPLUGIN_H

#include "facetrack/realtimetrack.h"
#include "facelib/mesh.h"
#include "facelib/facealigner.h"

class KinectSensorPlugin
{
private:
    unsigned char rgbBuffer[640 * 480 * 3];
    double depthBuffer[640 * 480];
    bool depthMask[640 * 480];
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

    void scanFace();

    void go();
    void off() {}
    void wait();
    void position();
    void capture();
    void deleteMesh();
    void align();
    bool isPositionInterupted() { return positionInterupted; }

    static bool isKinectPluggedIn();
};

#endif // KINECTSENSORPLUGIN_H
