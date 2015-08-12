#pragma once

#include "imagefilter.h"

namespace Face {
namespace LinAlg {

class FACECOMMON_EXPORTS ImageFilterFactory
{
public:
    static ImageFilter::Ptr create(const std::string &params);
    static std::vector<ImageFilter::Ptr> create(const std::string &params, const std::string &separator);
};

}
}
