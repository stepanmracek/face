#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include "common.h"

namespace Face {
namespace LinAlg {

class Histogram
{
public:
    double minValue;
    double maxValue;
    double mean;
    double stdDev;
    std::vector<double> histogramCounter;
    std::vector<double> histogramValues;

    Histogram() {}
    Histogram(const std::vector<double> &values, int bins, bool normalize, double minValue = 0, double maxValue = 0, bool ignoreOutliers = false);

    void savePlot(const std::string &path) const;
    Matrix plot() const;
};

}
}

#endif // HISTOGRAM_H
