#pragma once

#include <opencv/cv.h>

#include "common.h"

namespace Face {
namespace LinAlg {

class SVD
{
public:
    static Matrix procrustes(Matrix &A, Matrix &B)
    {
        Matrix M = A.t()*B;
        cv::SVD svd(M);

        Matrix R = svd.u * svd.vt;
        return R;
    }
};

}
}

