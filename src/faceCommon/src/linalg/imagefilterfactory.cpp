#include "faceCommon/linalg/imagefilterfactory.h"

#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>

#include "faceCommon/linalg/imagefilter.h"
#include "faceCommon/linalg/spatialhistogram.h"

using namespace Face::LinAlg;

ImageFilter::Ptr ImageFilterFactory::create(const std::string &params)
{
    Poco::StringTokenizer paramsList(params, "-");
    std::string filterName = paramsList[0];

    if (filterName == GaborFilterReal::name())
    {
        int f = Poco::NumberParser::parse(paramsList[1]);
        int o = Poco::NumberParser::parse(paramsList[2]);
        return new GaborFilterReal(f, o);
    }
    else if (filterName == GaborFilterImag::name())
    {
        int f = Poco::NumberParser::parse(paramsList[1]);
        int o = Poco::NumberParser::parse(paramsList[2]);
        return new GaborFilterImag(f, o);
    }
    else if (filterName == GaborFilterAbs::name())
    {
        int f = Poco::NumberParser::parse(paramsList[1]);
        int o = Poco::NumberParser::parse(paramsList[2]);
        return new GaborFilterAbs(f, o);
    }
    else if (filterName == GaussLaguerreFilterReal::name())
    {
        int s = Poco::NumberParser::parse(paramsList[1]);
        int n = Poco::NumberParser::parse(paramsList[2]);
        int k = Poco::NumberParser::parse(paramsList[3]);
        return new GaussLaguerreFilterReal(s, n, k);
    }
    else if (filterName == GaussLaguerreFilterImag::name())
    {
        int s = Poco::NumberParser::parse(paramsList[1]);
        int n = Poco::NumberParser::parse(paramsList[2]);
        int k = Poco::NumberParser::parse(paramsList[3]);
        return new GaussLaguerreFilterImag(s, n, k);
    }
    else if (filterName == GaussLaguerreFilterAbs::name())
    {
        int s = Poco::NumberParser::parse(paramsList[1]);
        int n = Poco::NumberParser::parse(paramsList[2]);
        int k = Poco::NumberParser::parse(paramsList[3]);
        return new GaussLaguerreFilterAbs(s, n, k);
    }
    else if (filterName == DifferenceOfGaussiansFilter::name())
    {
        int s1 = Poco::NumberParser::parse(paramsList[1]);
        int s2 = Poco::NumberParser::parse(paramsList[2]);
        return new DifferenceOfGaussiansFilter(s1, s2);
    }
    else if (filterName == EqualizeFilter::name())
    {
        return new EqualizeFilter();
    }
    else if (filterName == GaussianBlurFilter::name())
    {
        int s = Poco::NumberParser::parse(paramsList[1]);
        return new GaussianBlurFilter(s);
    }
    else if (filterName == ScaleFilter::name())
    {
        double s = Poco::NumberParser::parseFloat(paramsList[1]);
        return new ScaleFilter(s);
    }
    else if (filterName == SpatialHistogramWLD::name())
    {
        int x = Poco::NumberParser::parse(paramsList[1]);
        int y = Poco::NumberParser::parse(paramsList[2]);
        return new SpatialHistogramWLD(x, y);
    }
    else if (filterName == LBPFilter::name())
    {
        return new LBPFilter();
    }
    else if (filterName == HistogramFilter::name())
    {
        int gridSizeX = Poco::NumberParser::parse(paramsList[1]);
        int gridSizeY = Poco::NumberParser::parse(paramsList[2]);
        return new HistogramFilter(gridSizeX, gridSizeY);
    }
    else if (filterName == HistogramBinsFilter::name())
    {
        int gridSizeX = Poco::NumberParser::parse(paramsList[1]);
        int gridSizeY = Poco::NumberParser::parse(paramsList[2]);
        int binsCount = Poco::NumberParser::parse(paramsList[3]);
        return new HistogramBinsFilter(gridSizeX, gridSizeY, binsCount);
    }
    else if (filterName == ContrastEqualizationFilter::name())
    {
        return new ContrastEqualizationFilter();
    }
    else if (filterName == OrientedGradientsFilter::name())
    {
        return new OrientedGradientsFilter();
    }

    throw FACELIB_EXCEPTION("unknown filter " + filterName);
}

std::vector<ImageFilter::Ptr> ImageFilterFactory::create(const std::string &params, const std::string &separator)
{
    Poco::StringTokenizer items(params, separator);
    std::vector<ImageFilter::Ptr> result;
    for (const std::string &p : items)
    {
        if (p.empty()) continue;
        result.push_back(create(p));
    }
    return result;
}
