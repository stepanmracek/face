#ifndef TEST_H
#define TEST_H

#include <QDebug>
#include "linalg/common.h"
#include "linalg/matrixconverter.h"
#include "linalg/differenceofgaussians.h"

class Test
{
public:
    static void DoG()
    {
        int k1 = 3;
        int k2 = 35;

        Matrix img = Common::loadMatrix("/media/data/frgc/spring2004/zbin-aligned2/textureI/02463d652.gz");

        Matrix diff = DifferenceOfGaussians::dog(img, k1, k2, false);
        Matrix equalized = DifferenceOfGaussians::dog(img, k1, k2, true);

        cv::imshow("img", img);
        cv::imshow("diff", diff);
        cv::imshow("equalized", equalized);
        cv::waitKey();
    }
};

#endif // TEST_H
