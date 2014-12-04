#ifndef GUIDRAWER_H
#define GUIDRAWER_H

#include "softkinetic/ds325sensorfactory.h"

namespace Face {
namespace Sensors {
namespace SoftKinetic {

class GUIDrawer
{
public:
    static void draw(const IDS325Sensor::Output &output);
private:
    static std::string stateToText(IDS325Sensor::State state);
    static std::string positioningOutputToText(IDS325Sensor::PositioningOutput positioningOutput);
};

}
}
}

#endif // GUIDRAWER_H
