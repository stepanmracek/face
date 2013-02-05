#include "bioidprocess.h"

#include <QVector>
#include <QDebug>
#include <QStringList>

#include <cassert>
#include <opencv/cv.h>

#include "linalg/common.h"
#include "linalg/loader.h"
#include "linalg/matrixconverter.h"

void BioIDProcess::process(const char *path)
{
    QVector<Matrix> processedImages;
    QStringList imagesPaths = Loader::listFiles(path, "*.pgm", true);
    QStringList eyesPaths = Loader::listFiles(path, "*.eye", true);

    int n = imagesPaths.count();
    assert(n == eyesPaths.count());
    assert(n > 0);

    for (int i = 0; i < n; i++)
    {
        qDebug() << imagesPaths[i] << eyesPaths[i];

        Matrix fullImage = cv::imread(imagesPaths[i].toStdString());
        cv::namedWindow("img");
        cv::imshow("img", fullImage);
        cv::waitKey();

        break;
    }
}
