#ifndef TESTSVD_H
#define TESTSVD_H

#include <opencv/cv.h>

#include "linalg/common.h"
#include "linalg/svd.h"

class TestSVD
{
public:
    static void testProcrustes()
    {
        Matrix B = (cv::Mat_<double>(4,3) << 0,0,0, 1,0,0, 1,0,1, 0,0,1);
        Matrix A = (cv::Mat_<double>(4,3) << 0,-0.1,0, 1,-0.2,0, 1,0.1,1, 0,0.2,1);

        Matrix R = Face::LinAlg::SVD::procrustes(A, B);
        Matrix A2 = A*R;
        Face::LinAlg::Common::printMatrix(A2);
    }
};

#endif // TESTSVD_H
