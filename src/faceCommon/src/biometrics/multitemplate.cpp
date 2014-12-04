#include "faceCommon/biometrics/multitemplate.h"

using namespace Face::Biometrics;

MultiTemplate::MultiTemplate() :
    version(0),
    id(0)
{
}

MultiTemplate::MultiTemplate(int version, int id, const std::vector<Face::LinAlg::Vector> &featureVectors) :
    version(version),
    id(id),
    featureVectors(featureVectors)
{

}

MultiTemplate::MultiTemplate(const std::string &path)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    deserialize(storage);
}

MultiTemplate::MultiTemplate(cv::FileStorage &storage)
{
    deserialize(storage);
}

void MultiTemplate::serialize(cv::FileStorage &storage) const
{
    storage << "version" << version;
    storage << "id" << id;

    storage << "features" << "[";
    for (const Face::LinAlg::Vector &v : featureVectors)
    {
        storage << v;
    }
    storage << "]";
}

void MultiTemplate::deserialize(cv::FileStorage &storage)
{
    storage["version"] >> version;
    storage["id"] >> id;

    cv::FileNode vectorsNode = storage["features"];
    for (cv::FileNodeIterator it = vectorsNode.begin(); it != vectorsNode.end(); ++it)
    {
        Matrix m;
        (*it) >> m;
        featureVectors.push_back(Face::LinAlg::Vector(m));
    }
}

