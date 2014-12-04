#ifndef SETTINGS_H
#define SETTINGS_H

namespace Face
{
namespace Sensors
{
namespace SoftKinetic
{

struct Settings
{
    Settings() :
        consecutiveDetectsToStartPositioning(5),
        minimalFaceAreaToStartPositioning(2000),
        consecutiveNonDetectsToLeavePositioning(60),
        consecutiveOptimalDistanceToStartCapturing(20),
        captureFrames(20),
        minimalDistance(0.3),
        maximalDistance(0.45)
    { }

    int consecutiveDetectsToStartPositioning;
    int minimalFaceAreaToStartPositioning;
    int consecutiveNonDetectsToLeavePositioning;
    int consecutiveOptimalDistanceToStartCapturing;
    int captureFrames;
    double minimalDistance;
    double maximalDistance;
    double minimalTilt;
    double maximalTilt;

};

}
}
}

#endif // SETTINGS_H
