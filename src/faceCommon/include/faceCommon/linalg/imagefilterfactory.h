#ifndef IMAGEFILTERFACTORY_H
#define IMAGEFILTERFACTORY_H

#include "imagefilter.h"

namespace Face {
namespace LinAlg {

class ImageFilterFactory
{
public:
    static ImageFilter::Ptr create(const std::string &params);
    static std::vector<ImageFilter::Ptr> create(const std::string &params, const std::string &separator);
};

}
}

#endif // IMAGEFILTERFACTORY_H
