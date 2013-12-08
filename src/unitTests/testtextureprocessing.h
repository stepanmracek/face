#ifndef TESTTEXTUREPROCESSING_H
#define TESTTEXTUREPROCESSING_H

#include "linalg/loader.h"
#include "biometrics/biodataprocessing.h"
#include "biometrics/zpcacorrw.h"
#include "biometrics/evaluation.h"

#include <opencv2/opencv.hpp>

class TestTextureProcessing
{
private:
    static cv::Mat process(cv::Mat &input)
    {
        //cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(16);
        //clahe->apply(input, input);
        //cv::GaussianBlur(input, input, cv::Size(3,3), 0);
        //return input;

        cv::equalizeHist(input, input);
        return input;
    }

    static Vector toVector(cv::Mat &input)
    {
        cv::Mat resized;
        cv::resize(input, resized, cv::Size(input.cols/2, input.rows/2));
        Matrix m = MatrixConverter::grayscaleImageToDoubleMatrix(resized);
        return MatrixConverter::matrixToColumnVector(m);
    }

    static int getId(const QString &baseName)
    {
        return baseName.split("d")[0].toInt();
    }

public:
    static void testClahe()
    {
        QString dir = "/home/stepo/data/frgc/spring2004/zbin-aligned2/textureI/";
        QVector<QString> names = Loader::listFiles(dir, "*.png", BaseFilename);

        QVector<int> classes;
        QVector<Vector> vectors;
        foreach (const QString &baseName, names)
        {
            cv::Mat img = cv::imread((dir + baseName + ".png").toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
            img = process(img);

            vectors << toVector(img);
            classes << getId(baseName);
        }

        QList<QVector<Vector> > vecsInClusters;
        QList<QVector<int> > classesInClusters;
        BioDataProcessing::divideToNClusters(vectors, classes, 5, vecsInClusters, classesInClusters);

        ZPCACorrW pca(vecsInClusters[0], 0.995, vecsInClusters[1]);
        Evaluation e(vecsInClusters[2], classesInClusters[2], pca.extractor, pca.metric);
        qDebug() << e.eer;
    }
};

#endif // TESTTEXTUREPROCESSING_H
