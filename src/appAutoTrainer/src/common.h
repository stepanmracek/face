#ifndef TRAINER_COMMON_H
#define TRAINER_COMMON_H

#include <string>

#include "faceCommon/facedata/facealigner.h"

namespace Face {
namespace AutoTrainer {

class Common
{
public:
    static void loadMeshes(const std::string &dir, const Face::FaceData::FaceAligner &aligner,
                           std::vector<int> &ids, std::vector<Face::FaceData::Mesh> &meshes, int icpIterations,
                           int smoothIterations, float smoothCoef, const std::string &idSeparator);
};

}
}

#endif // TRAINER_COMMON_H
