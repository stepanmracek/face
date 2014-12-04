#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include "common.h"
#include "faceCommon/facedata/mesh.h"

namespace Face {
namespace LinAlg {

class Serialization
{
public:
    static void serializeVectorOfPointclouds(std::vector<Face::FaceData::VectorOfPoints> &data, const std::string &path);
    static std::vector<Face::FaceData::VectorOfPoints> readVectorOfPointclouds(const std::string &path);
};

}
}

#endif // SERIALIZATION_H
