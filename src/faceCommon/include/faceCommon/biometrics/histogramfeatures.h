#ifndef HISTOGRAMFEATURES_H
#define HISTOGRAMFEATURES_H


#include "faceCommon/linalg/vector.h"
#include "faceCommon/linalg/histogram.h"
#include "faceCommon/facedata/map.h"

namespace Face {
namespace Biometrics {

class HistogramFeatures
{
public:
    std::vector<Face::LinAlg::Histogram> histograms;

    HistogramFeatures(const FaceData::Map &depthmap, unsigned int stripes, unsigned int binsPerStripe,
                      double minValue = 0, double maxValue = 0);

    HistogramFeatures(const FaceData::Map &depthmap, unsigned int stripes, unsigned int binsPerStripe,
                      const std::vector<double> &minValues, const std::vector<double> &maxValues);

    HistogramFeatures(const ImageGrayscale &depthmap, unsigned int stripes, unsigned int binsPerStripe);

    Face::LinAlg::Vector toVector();

    static std::pair<std::vector<double>, std::vector<double> > trainMinMaxValues(const std::vector<FaceData::Map> &depthmaps,
                                                                      int stripes, double stdMultiplier = 2);
};

}
}

#endif // HISTOGRAMFEATURES_H
