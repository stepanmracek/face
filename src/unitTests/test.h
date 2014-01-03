#ifndef TEST_H
#define TEST_H

#include <QDebug>
#include "linalg/common.h"
#include "linalg/matrixconverter.h"

class Test
{
public:
    static void DoG()
    {
        Matrix img = Common::loadMatrix("/media/data/frgc/spring2004/zbin-aligned2/textureE/02463d652.gz");
        qDebug() << img.rows;

        Matrix blurred1;
        cv::GaussianBlur(img, blurred1, cv::Size(9,9), 0);

        Matrix blurred2;
        cv::GaussianBlur(img, blurred2, cv::Size(15,15), 0);

        Matrix diff = blurred2 - blurred1;
        double min, max;
        cv::minMaxIdx(diff, &min, &max);
        diff = (diff-min)/(max-min);

        Matrix equalized = MatrixConverter::equalize(diff);

        cv::imshow("img", img);
        cv::imshow("blurred1", blurred1);
        cv::imshow("blurred2", blurred2);
        cv::imshow("diff", diff);
        cv::imshow("equalized", equalized);
        cv::waitKey();
    }
};

#endif // TEST_H
