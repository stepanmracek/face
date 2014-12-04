#ifndef MULTITEMPLATE_H
#define MULTITEMPLATE_H

#include "faceCommon/linalg/vector.h"
#include "faceCommon/linalg/iserializable.h"

namespace Face {
namespace Biometrics {

class MultiTemplate : public Face::LinAlg::ISerializable
{
public:
    int version;
    int id;
    std::vector<Face::LinAlg::Vector> featureVectors;

    MultiTemplate();
    MultiTemplate(int version, int id, const std::vector<Face::LinAlg::Vector> &featureVectors);
    MultiTemplate(const std::string &path);
    MultiTemplate(cv::FileStorage &storage);

    void serialize(cv::FileStorage &storage) const;
    void deserialize(cv::FileStorage &storage);
};

}
}

#endif // MULTITEMPLATE_H
