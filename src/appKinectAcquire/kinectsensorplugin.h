#ifndef KINECTSENSORPLUGIN_H
#define KINECTSENSORPLUGIN_H

#include "facetrack/realtimetrack.h"
#include "facelib/mesh.h"

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

    enum PositionResponse { NoData, Ok, MoveCloser, MoveFar };
    PositionResponse depthStats();
    void putText(ImageGrayscale &img, const char *text);

public:
    enum State { Off, Waiting, Positioning, Capturing };
    KinectSensorPlugin(const QString &faceDetectorPath);
    bool position();
    Mesh *scan();
};

#endif // KINECTSENSORPLUGIN_H
