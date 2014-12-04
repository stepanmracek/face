#ifndef ISERIALIZABLE_H
#define ISERIALIZABLE_H

#include <opencv2/opencv.hpp>

namespace Face
{
namespace LinAlg
{

class ISerializable
{
public:
    virtual void serialize(cv::FileStorage &storage) const = 0;
    virtual void deserialize(cv::FileStorage &storage) = 0;
    virtual ~ISerializable() {}
};

}
}

#endif // ISERIALIZABLE_H
