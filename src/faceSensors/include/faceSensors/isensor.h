#ifndef ISENSOR_H
#define ISENSOR_H

#include "faceCommon/facedata/mesh.h"

namespace Face {
namespace Sensors {

class ISensor
{
public:
    typedef cv::Ptr<ISensor> Ptr;

    virtual void scan() = 0;
    virtual Face::FaceData::Mesh &mesh() = 0;
    virtual ~ISensor() {}
};

}
}

#endif // ISENSOR_H
