#pragma once

#include "mesh.h"
#include "landmarks.h"

namespace Face {
namespace FaceData {

class FACECOMMON_EXPORTS FaceFeaturesAnotation
{
public:  
    static void anotateFromFiles(const std::string &dirPath);

    static Landmarks anotate(Mesh &mesh, bool &success);
};

}
}
