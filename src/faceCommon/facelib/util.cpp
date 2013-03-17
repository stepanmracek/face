#include <math.h>
#include "util.h"

int Util::maxIndexInArray(double* array, int start, int end)
{
    int maxindex = -1;
    double maxValue = -1e300;
    for (int i = start; i < end; i++)
    {
        if (array[i] > maxValue)
        {
            maxValue = array[i];
            maxindex = i;
        }
    }
    return maxindex;
}

int Util::minIndexInArray(double *array, int start, int end)
{
    int minindex = -1;
    double minValue = 1e300;
    for (int i = start; i < end; i++)
    {
        if (array[i] < minValue)
        {
            minValue = array[i];
            minindex = i;
        }
    }
    return minindex;
}

double Util::VectorSize(double *array, int length)
{
    double sum = 0.0;
    for (int i = 0; i < length; i++)
    {
        sum += (array[i]*array[i]);
    }

    return sqrt(sum);
}

void Util::NormalizeVector(double *array, int length)
{
    double size = 1/VectorSize(array, length);
    for (int i = 0; i < length; i++)
    {
        array[i] = array[i]*size;
    }
}

double Util::VectorMult(double *vec1, double *vec2, int length)
{
    double sum = 0.0;
    for (int i = 0; i < length; i++)
    {
        sum += (vec1[i]*vec2[i]);
    }

    return sum;
}

double Util::angle(double *vec1, double *vec2, int length)
{
    NormalizeVector(vec1, length);
    NormalizeVector(vec2, length);

    return acos(VectorMult(vec1,vec2, length));
}
