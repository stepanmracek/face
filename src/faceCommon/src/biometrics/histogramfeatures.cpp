#include "faceCommon/biometrics/histogramfeatures.h"

using namespace Face::Biometrics;

HistogramFeatures::HistogramFeatures(const Face::FaceData::Map &depthmap, unsigned int stripes, unsigned int binsPerStripe, double minValue, double maxValue)
{
    std::vector<std::vector<double> > valuesInStripes(stripes);
    for (int y = 0; y < depthmap.h; y++)
    {
        unsigned int histogramIndex = ((double)y)/depthmap.h * stripes;
        if (histogramIndex == stripes) histogramIndex--;

        for (int x = 0; x < depthmap.w; x++)
        {
            if (!depthmap.isSet(x, y)) continue;

            valuesInStripes[histogramIndex].push_back(depthmap.get(x, y));
        }
    }

    for (unsigned int i = 0; i < stripes; i++)
    {
        histograms.push_back(Face::LinAlg::Histogram(valuesInStripes[i], binsPerStripe, true, minValue, maxValue));
    }
}

HistogramFeatures::HistogramFeatures(const Face::FaceData::Map &depthmap, unsigned int stripes, unsigned int binsPerStripe,
                                     const std::vector<double> &minValues, const std::vector<double> &maxValues)
{
    if ((stripes != minValues.size()) || (stripes == maxValues.size()))
        throw FACELIB_EXCEPTION("stripes and min/maxValues count mismatch");

    std::vector<std::vector<double> > valuesInStripes(stripes);
    for (int y = 0; y < depthmap.h; y++)
    {
        unsigned int histogramIndex = ((double)y)/depthmap.h * stripes;
        if (histogramIndex == stripes) histogramIndex--;

        for (int x = 0; x < depthmap.w; x++)
        {
            if (!depthmap.isSet(x, y)) continue;

            valuesInStripes[histogramIndex].push_back(depthmap.get(x, y));
        }
    }

    for (unsigned int i = 0; i < stripes; i++)
    {
        histograms.push_back(Face::LinAlg::Histogram(valuesInStripes[i], binsPerStripe, true, minValues[i], maxValues[i]));
    }
}

HistogramFeatures::HistogramFeatures(const ImageGrayscale &depthmap, unsigned int stripes, unsigned int binsPerStripe)
{
    std::vector<std::vector<double> > valuesInStripes(stripes);
    for (int y = 0; y < depthmap.rows; y++)
    {
        unsigned int histogramIndex = (((double)y)/depthmap.rows) * stripes;
        if (histogramIndex == stripes) histogramIndex--;

        for (int x = 0; x < depthmap.cols; x++)
        {
            valuesInStripes[histogramIndex].push_back(depthmap(y, x));
        }
    }

    for (unsigned int i = 0; i < stripes; i++)
    {
        histograms.push_back(Face::LinAlg::Histogram(valuesInStripes[i], binsPerStripe, true, 0, 255));
    }
}

Face::LinAlg::Vector HistogramFeatures::toVector()
{
    std::vector<double> allValues;
    for (unsigned int i = 0; i < histograms.size(); i++)
    {
        const auto &h = histograms[i].histogramCounter;
        allValues.insert(allValues.end(), h.begin(), h.end());
    }

    return Face::LinAlg::Vector(allValues);
}

std::pair<std::vector<double>, std::vector<double> > HistogramFeatures::trainMinMaxValues(
        const std::vector<Face::FaceData::Map> &depthmaps, int stripes, double stdMultiplier)
{
    std::pair<std::vector<double>, std::vector<double> > result;
    std::vector<std::vector<double> > valuesInStripes(stripes);

    for (const FaceData::Map &depthmap : depthmaps)
    {
        for (int y = 0; y < depthmap.h; y++)
        {
            int stripeIndex = ((double)y)/depthmap.h * stripes;
            if (stripeIndex == stripes) stripeIndex--;

            for (int x = 0; x < depthmap.w; x++)
            {
                if (!depthmap.isSet(x, y)) continue;

                valuesInStripes[stripeIndex].push_back(depthmap.get(x, y));
            }
        }
    }

    int stripeIndex = 0;
    for (const std::vector<double> &stripeValues : valuesInStripes)
    {
        Face::LinAlg::Vector v(stripeValues);
        double mean = v.meanValue();
        double std = v.stdDeviation();
        double min = v.minValue();
        double max = v.maxValue();
        std::cout << stripeIndex << " " << mean << " " << std << " "
                  << (mean - stdMultiplier*std) << " " << (mean + stdMultiplier*std)
                  << " " << min << " " << max;

        result.first.push_back(mean - stdMultiplier*std);
        result.second.push_back(mean + stdMultiplier*std);

        Face::LinAlg::Histogram h(stripeValues, 50, true);
        Face::LinAlg::Common::savePlot(h.histogramValues, h.histogramCounter, std::to_string(stripeIndex));

        stripeIndex++;
    }

    return result;
}
