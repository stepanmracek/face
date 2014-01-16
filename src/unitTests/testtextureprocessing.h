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

    static Face::LinAlg::Vector toVector(cv::Mat &input)
    {
        cv::Mat resized;
        cv::resize(input, resized, cv::Size(input.cols/2, input.rows/2));
        Matrix m = Face::LinAlg::MatrixConverter::grayscaleImageToDoubleMatrix(resized);
        return Face::LinAlg::MatrixConverter::matrixToColumnVector(m);
    }

    static int getId(const QString &baseName)
    {
        return baseName.split("d")[0].toInt();
    }

public:
    static void testClahe()
    {
        QString dir = "/home/stepo/data/frgc/spring2004/zbin-aligned2/textureI/";
        QVector<QString> names = Face::LinAlg::Loader::listFiles(dir, "*.png", Face::LinAlg::Loader::BaseFilename);

        QVector<int> classes;
        QVector<Face::LinAlg::Vector> vectors;
        foreach (const QString &baseName, names)
        {
            cv::Mat img = cv::imread((dir + baseName + ".png").toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
            img = process(img);

            vectors << toVector(img);
            classes << getId(baseName);
        }

        QList<QVector<Face::LinAlg::Vector> > vecsInClusters;
        QList<QVector<int> > classesInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters(vectors, classes, 5, vecsInClusters, classesInClusters);

        Face::Biometrics::ZPCACorrW pca(vecsInClusters[0], 0.995, vecsInClusters[1]);
        Face::Biometrics::Evaluation e(vecsInClusters[2], classesInClusters[2], pca.extractor, pca.metric);
        qDebug() << e.eer;
    }
};

#endif // TESTTEXTUREPROCESSING_H
