#pragma once

#include "faceCommon/linalg/common.h"

namespace Face {
namespace FaceData {

class MaskedVector
{
public:
    int n;
    std::vector<bool> flags;
    std::vector<double> values;

    MaskedVector(int len, double initValue, bool initFlags)
    {
        n = len;
        flags = std::vector<bool>(len, initFlags);
        values = std::vector<double>(len, initValue);
    }

    static MaskedVector diff(MaskedVector &a, MaskedVector &b);

    void set(int i, double value)
    {
        flags[i] = true;
        values[i] = value;
    }

    int flagCount()
    {
        int count = 0;
        for (int i = 0; i < n; i++)
        {
            if (flags[i])
            {
                count++;
            }
        }
        return count;
    }

    MaskedVector derivate();

    bool save(const std::string &filename);
    bool savePlot(const std::string &filename, bool append = false);

    int maxIndex();
    double mean();
    double max();
    double median();
};

}
}
