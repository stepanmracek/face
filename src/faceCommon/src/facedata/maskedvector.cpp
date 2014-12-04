#include "faceCommon/facedata/maskedvector.h"

#include <fstream>

using namespace Face::FaceData;

MaskedVector MaskedVector::diff(MaskedVector &a, MaskedVector &b)
{
    MaskedVector c(a.n, 0.0, false);
    for (int i = 0; i < a.n; i++)
    {
        if (a.flags[i] && b.flags[i])
        {
            c.set(i, a.values[i] - b.values[i]);
        }
    }
    return c;
}

bool MaskedVector::save(const std::string &filename)
{
    std::ofstream out(filename);
    if (out.is_open())
    {
        out << n << std::endl;
        for (int i = 0; i < n; i++)
        {
            out << values[i] << " " << flags[i] << std::endl;
        }
        out.flush();
        out.close();
        return true;
    }
    else
    {
        return false;
    }
}

bool MaskedVector::savePlot(const std::string &filename, bool append)
{
    std::ofstream out;
    if (append)
        out.open(filename, std::ofstream::app);
    else
        out.open(filename);

    if (!out.is_open()) return false;

    for (int i = 0; i < n; i++)
    {
        if (flags[i])
        {
            out << i << " " << values[i] << std::endl;
        }
    }
    out << std::endl;
    out.flush();
    out.close();
    return true;
}


double MaskedVector::mean()
{
    double sum = 0;
    int count = 0;
    for (int i = 0; i < n; i++)
    {
        if (flags[i])
        {
            sum += values[i];
            count++;
        }
    }
    return sum/count;
}

int MaskedVector::maxIndex()
{
    int index = -1;
    double max = -1e300;
    for (int i = 0; i < n; i++)
    {
        if (flags[i] && values[i] > max)
        {
            max = values[i];
            index = i;
        }
    }
    return index;
}

double MaskedVector::max()
{
    return values[maxIndex()];
}

double MaskedVector::median()
{
    std::vector<double> list;
    for (int i = 0; i < n; i++)
    {
        if (flags[i])
        {
            list.push_back(values[i]);
        }
    }

    std::sort(list.begin(), list.end());
    int len = list.size();
    if (len % 2 == 1)
    {
        return list[(len-1)/2];
    }
    else
    {
        return (list[len/2] + list[(len-2)/2])/2;
    }
}

MaskedVector MaskedVector::derivate()
{
    MaskedVector result(n, 0, false);
    for (int i = 1; i < (n-1); i++)
    {
        if (flags[i-1] && flags[i] && flags[i+1])
        {
            double d1 = values[i] - values[i-1];
            double d2 = values[i+1] - values[i];
            result.set(i, (d1+d2)*0.5);
        }
        else if (flags[i-1] && flags[i])
        {
            result.set(i, values[i] - values[i-1]);
        }
        else if (flags[i] && flags[i+1])
        {
            result.set(i, values[i+1] - values[i]);
        }
    }
    return result;
}
