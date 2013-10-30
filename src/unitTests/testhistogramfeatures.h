#ifndef TESTHISTOGRAMFEATURES_H
#define TESTHISTOGRAMFEATURES_H

#include <QString>

#include "biometrics/histogramfeatures.h"

class TestHistogramFeatures
{
public:
    static void testFeaturesGeneration(const QString &frgcPath)
    {
        ImageGrayscale full1 = cv::imread((frgcPath + "zbin-aligned/depth2/02463d652.png").toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        ImageGrayscale full2 = cv::imread((frgcPath + "zbin-aligned/depth2/02463d654.png").toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        ImageGrayscale full3 = cv::imread((frgcPath + "zbin-aligned/depth2/04200d74.png").toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        ImageGrayscale cropped1 = full1(cv::Rect(40, 20, 220, 180));
        ImageGrayscale cropped2 = full2(cv::Rect(40, 20, 220, 180));
        ImageGrayscale cropped3 = full3(cv::Rect(40, 20, 220, 180));
        HistogramFeatures features1(cropped1, 6, 6);
        HistogramFeatures features2(cropped2, 6, 6);
        HistogramFeatures features3(cropped3, 6, 6);
        /*for (int i = 0; i < 6; i++)
        {
            qDebug() << features1.histograms[i].minValue << features1.histograms[i].maxValue;
            Common::savePlot(features1.histograms[i].histogramValues, features1.histograms[i].histogramCounter, "histogram-stripe-" + QString::number(i));
        }*/

        Vector vec1 = features1.toVector();
        Vector vec2 = features2.toVector();
        Vector vec3 = features3.toVector();
        vec1.toFile("histogram-allstripes1");
        vec2.toFile("histogram-allstripes2");
        vec3.toFile("histogram-allstripes3");
    }
};

#endif // TESTHISTOGRAMFEATURES_H
