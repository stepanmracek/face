#ifndef TESTICA_H
#define TESTICA_H

#endif // TESTICA_H


#include <QVector>
#include <QList>
#include <QDebug>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "linalg/common.h"
#include "linalg/dataviz.h"
#include "linalg/random.h"
#include "linalg/ica.h"
#include "linalg/vector.h"
#include "biometrics/biodataprocessing.h"
#include "linalg/loader.h"
#include "linalg/icaofpca.h"
#include "biometrics/template.h"

void onTestICAChange(int newVal, void *userData);

class TestICA
{
public:
    static void testDataGeneration()
    {
        QVector<double> mean1; mean1 << -3 << 0;
        QVector<double> mean2; mean2 << 3 << 0;
        QVector<double> sigma; sigma << 1 << 10;
        QVector<Vector> class1 = Random::gauss(mean1, sigma, 500);
        QVector<Vector> class2 = Random::gauss(mean2, sigma, 500);

        class1 << class2; // copy second class to first one
        ICA ica(class1, 2);
        ica.setModes(1);

        Vector test(2);
        test(0) = -3; test(1) = 0;
        Vector projected = ica.project(test);
        qDebug() << test.toQVector() << projected.toQVector();

        test(0) = -3; test(1) = 5;
        projected = ica.project(test);
        qDebug() << test.toQVector() << projected.toQVector();

        test(0) = -3; test(1) = -5;
        projected = ica.project(test);
        qDebug() << test.toQVector() << projected.toQVector();

        test(0) = 3; test(1) = 0;
        projected = ica.project(test);
        qDebug() << test.toQVector() << projected.toQVector();

        test(0) = 3; test(1) = 5;
        projected = ica.project(test);
        qDebug() << test.toQVector() << projected.toQVector();

        test(0) = 3; test(1) = -5;
        projected = ica.project(test);
        qDebug() << test.toQVector() << projected.toQVector();
    }

    static void saveVec(Vector &vec, int id, int scan, const QString& suffix)
    {
        QString idS = ((id < 10)?"0":"")+QString::number(id);
        QString scanS = ((scan < 10)?"0":"")+QString::number(scan);
        QString path = "ica/" + idS + "-" + scanS + suffix;

        vec.toFile(path);
    }

    ICAofPCA ica;
    int imgWidth;
    Matrix parameters;
    char *winname;
    int trackbarValues[10];
    void DeprojectGermanyThermoImages()
    {
        QVector<Vector> allImages;
        QVector<int> allClasses;
        Loader::loadImages("/home/stepo/SVN/disp-stepan-mracek/databases/thermo/germany", allImages, &allClasses, "*.png", "-", false);
        imgWidth = 50;
        winname = "ICA Facespace";
        cv::namedWindow(winname);

        // Divide data
        QList<QVector<Vector> > images;
        QList<QVector<int> > classes;
        BioDataProcessing::divideTemplatesToClusters(allImages, allClasses, 15, images, classes);

        ica.learn(images[0], 0.99, 20);

        cv::Size size(imgWidth*4, imgWidth*4);
        for (int i = 0; i < ica.ica.W.rows; i++)
        {
            Matrix in = ica.ica.W.row(i);
            in = in.t();
            Vector out = ica.pca.backProject(in);
            Matrix diff = out - ica.pca.cvPca.mean;
            out = diff;
            out.normalizeComponents();
            Matrix img = MatrixConverter::columnVectorToMatrix(out, imgWidth);
            img = img*255;

            cv::resize(img, img, size);
            QString filename("icaVector"+QString::number(i)+".png");
            cv::imwrite(filename.toStdString(), img);
        }

        for (int i = 0; i < 10; i++)
        {
            trackbarValues[i] = 5;
            std::string tbname = QString::number(i).toStdString();
            cv::createTrackbar(tbname, winname, &(trackbarValues[i]), 10, onTestICAChange, this);
        }

        parameters = Matrix::zeros(20, 1);
        Vector backProjected = ica.backProject(parameters);
        Matrix image = MatrixConverter::columnVectorToMatrix(backProjected, imgWidth);
        cv::resize(image, image, size);

        cv::imshow(winname, image);
        cv::waitKey(0);
    }

    static void ConvertGermanyThermoImages()
    {
        QVector<Vector> allImages;
        QVector<int> allClasses;
        Loader::loadImages("/home/stepo/SVN/disp-stepan-mracek/databases/thermo/germany", allImages, &allClasses, "*.png", "-", true);

        // Divide data
        QList<QVector<Vector> > images;
        QList<QVector<int> > classes;
        BioDataProcessing::divideTemplatesToClusters(allImages, allClasses, 15, images, classes);

        ICAofPCA ica(images[0], 0.99, 20);

        // Create templates
        int scan = 1;
        for (int c = 0; c < images.count(); c++)
        {
            for (int i = 0; i < images[c].count(); i++)
            {
                int id = classes[c][i];
                Vector in = ica.pca.project(images[c][i]);
                Vector out = ica.ica.project(in);

                saveVec(in, id, scan, "-in");
                saveVec(out, id, scan, "-out");

                scan++;
                if (scan == 11)
                    scan = 1;
            }
        }

        Common::printMatrix(ica.ica.EDET);
    }
};

void onTestICAChange(int, void* userData)
{
    TestICA *test = (TestICA*)userData;
    for (int i = 0; i < 10; i++)
    {
        std::string tbname = QString::number(i).toStdString();
        double value = cv::getTrackbarPos(tbname, test->winname);
        value = (value-5.0)/2.0; // * (3*sqrt(test->pca.cvPca.eigenvalues(i)));
        test->parameters(i) = value;
    }

    Vector vec = test->ica.backProject(test->parameters);
    Matrix image = MatrixConverter::columnVectorToMatrix(vec, test->imgWidth);
    cv::Size size(image.rows*4, image.cols*4);
    cv::resize(image, image, size);
    cv::imshow(test->winname, image);
}
