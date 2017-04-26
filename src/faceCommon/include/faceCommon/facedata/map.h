#pragma once

#include <cmath>
#include <opencv/cv.h>

#include "maskedvector.h"
#include "faceCommon/linalg/common.h"
#include "faceCommon/faceCommon.h"

namespace Face {
namespace FaceData {

class MaskedVector;
class FACECOMMON_EXPORTS Map;
class FACECOMMON_EXPORTS Map
{
public:

    Matrix values;
    cv::Mat_<char> flags;
    int w;
    int h;

    Map();
    void init(int _w, int _h);
    Map(int w, int h);
    Map(const Map &src);
    Map(const std::string &path);

    std::vector<double> getUsedValues() const;

    void set(int x, int y, double v)
    {
        flags(y,x) = 1;
        values(y,x) = v;
    }

    void setAll(double v);

    void unset(int x, int y)
    {
        flags(y, x) = 0;
        values(y, x) = 0.0;
    }

    void unsetAll();

    bool isSet(int x, int y) const
    {
        if (!isValidCoord(x, y)) return false;
        return flags(y, x);
    }

    double getSafe(int x, int y, double safeValue) const
    {
        if (isSet(x, y))
            return values(y, x);
        else
            return safeValue;
    }

    double get(int x, int y, bool *success = 0) const
    {
        if (success != 0)
        {
            if (!isValidCoord(x, y) || !flags(y, x))
            {
                *success = false;
                return 0;
            }
            *success = true;
        }
        return values(y, x);
    }

    bool has8neigbours(int x, int y) const
    {
        if (x == 0 || x >= (w-1) || y == 0 || y >= (h-1)) return false;

        if (!flags(y  ,x-1)) return false;
        if (!flags(y  ,x+1)) return false;

        if (!flags(y+1,x-1)) return false;
        if (!flags(y+1,x  )) return false;
        if (!flags(y+1,x+1)) return false;

        if (!flags(y-1,x-1)) return false;
        if (!flags(y-1,x  )) return false;
        if (!flags(y-1,x+1)) return false;

        return true;
    }

    bool isValidCoord(int x, int y) const
    {
        return (x < w) && (x >= 0) && (y < h) && (y >= 0);
    }

    void levelSelect(double zLevel);

    void bandPass(double minValue, double maxValue, bool unsetBelowMin, bool unsetAboveMax);

    void erode(int kernelSize);

    void minMax(double &min, double &max) const;

    void add(const Map &other);

    void linearTransform(double multiply, double add);

    MaskedVector horizontalProfile(int y) const;

    MaskedVector verticalProfile(int x) const;

    MaskedVector meanVerticalProfile() const;

    MaskedVector maxVerticalProfile() const;

    MaskedVector medianVerticalProfile() const;

    MaskedVector horizontalPointDensity(int y, int stripeWidth) const;

    Map densityMap(int kernelSize, bool fromCenter) const;

    void maxIndex(int &x, int &y) const;

    Matrix toMatrix(double voidValue = 0.0, double min = 0.0, double max = 0.0) const;

    void getCropParams(int &startx, int &width, int &starty, int &height) const;

    Map subMap(int startx, int width, int starty, int height) const;

    static Map fromMatrix(Matrix &matrix, double voidValue = 0.0);

    void applyFilter(const Matrix &kernel, int times = 1, bool checkSum = false);

    void applyCvGaussBlur(int size, int times = 1);

    void serialize(const std::string &path);
};

}
}
