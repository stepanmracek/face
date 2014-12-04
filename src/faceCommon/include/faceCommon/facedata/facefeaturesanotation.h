#ifndef FACEFEATURESANOTATION_H
#define FACEFEATURESANOTATION_H

#include "mesh.h"
#include "landmarks.h"

namespace Face {
namespace FaceData {

class FaceFeaturesAnotation
{
public:  
    static void anotateFromFiles(const std::string &dirPath);

    static Landmarks anotate(Mesh &mesh, bool &success);
};

}
}

#endif // FACEFEATURESANOTATION_H
