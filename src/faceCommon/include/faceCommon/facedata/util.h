#ifndef UTIL_H
#define UTIL_H

#include "faceCommon/linalg/common.h"

namespace Face {
namespace FaceData {

class Util
{
public:
    static int maxIndexInArray(double* array, int start, int end);
    static int minIndexInArray(double* array, int start, int end);

    static double VectorMult(double *vec1, double *vec2, int length);
    static double VectorSize(double *array, int length);
    static void NormalizeVector(double *array, int length);
    static double angle(double *vec1, double *vec2, int length);
};

}
}

#endif // UTIL_H
