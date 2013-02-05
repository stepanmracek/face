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
        QVector<Matrix> class1 = Random::gauss(mean1, sigma, 500);
        QVector<Matrix> class2 = Random::gauss(mean2, sigma, 500);

        class1 << class2; // copy second class to first one
        ICA ica(class1, 2);
        ica.setModes(1);

        Matrix test = Matrix::ones(2, 1);
        test(0) = -3; test(1) = 0;
        Matrix projected = ica.project(test);
        qDebug() << Vector::toQVector(test) <<  Vector::toQVector(projected);

        test(0) = -3; test(1) = 5;
        projected = ica.project(test);
        qDebug() << Vector::toQVector(test) <<  Vector::toQVector(projected);

        test(0) = -3; test(1) = -5;
        projected = ica.project(test);
        qDebug() << Vector::toQVector(test) <<  Vector::toQVector(projected);

        test(0) = 3; test(1) = 0;
        projected = ica.project(test);
        qDebug() << Vector::toQVector(test) <<  Vector::toQVector(projected);

        test(0) = 3; test(1) = 5;
        projected = ica.project(test);
        qDebug() << Vector::toQVector(test) <<  Vector::toQVector(projected);

        test(0) = 3; test(1) = -5;
        projected = ica.project(test);
        qDebug() << Vector::toQVector(test) <<  Vector::toQVector(projected);
    }

    static void saveVec(Matrix &m, int id, int scan, const QString& suffix)
    {
        QString idS = ((id < 10)?"0":"")+QString::number(id);
        QString scanS = ((scan < 10)?"0":"")+QString::number(scan);
        QString path = "ica/" + idS + "-" + scanS + suffix;

        Vector::toFile(m, path);
    }

    ICAofPCA ica;
    int imgWidth;
    Matrix parameters;
    char *winname;
    int trackbarValues[10];
    void DeprojectGermanyThermoImages()
    {
        QVector<Matrix> allImages;
        QVector<int> allClasses;
        Loader::loadImages("/home/stepo/SVN/disp-stepan-mracek/databases/thermo/germany", allImages, &allClasses, "*.png", true, "-", false);
        imgWidth = 50;
        winname = "ICA Facespace";
        cv::namedWindow(winname);

        // Divide data
        QList<QVector<Matrix> > images;
        QList<QVector<int> > classes;
        BioDataProcessing::divide(allImages, allClasses, 15, images, classes);

        ica.learn(images[0], 0.99, 20);

        cv::Size size(imgWidth*4, imgWidth*4);
        for (int i = 0; i < ica.ica.W.rows; i++)
        {
            Matrix in = ica.ica.W.row(i);
            in = in.t();
            Matrix out = ica.pca.backProject(in);
            out = out - ica.pca.cvPca.mean;
            out = MatrixConverter::columnVectorToMatrix(out, imgWidth);
            out = Vector::normalizeComponents(out);
            out = out*255;

            cv::resize(out, out, size);
            QString filename("icaVector"+QString::number(i)+".png");
            cv::imwrite(filename.toStdString(), out);
        }

        for (int i = 0; i < 10; i++)
        {
            trackbarValues[i] = 5;
            std::string tbname = QString::number(i).toStdString();
            cv::createTrackbar(tbname, winname, &(trackbarValues[i]), 10, onTestICAChange, this);
        }

        parameters = Matrix::zeros(20, 1);
        Matrix backProjected = ica.backProject(parameters);
        backProjected = MatrixConverter::columnVectorToMatrix(backProjected, imgWidth);
        cv::resize(backProjected, backProjected, size);

        cv::imshow(winname, backProjected);
        cv::waitKey(0);
    }

    static void ConvertGermanyThermoImages()
    {
        QVector<Matrix> allImages;
        QVector<int> allClasses;
        Loader::loadImages("/home/stepo/SVN/disp-stepan-mracek/databases/thermo/germany", allImages, &allClasses, "*.png", true, "-", true);

        // Divide data
        QList<QVector<Matrix> > images;
        QList<QVector<int> > classes;
        BioDataProcessing::divide(allImages, allClasses, 15, images, classes);

        ICAofPCA ica(images[0], 0.99, 20);

        // Create templates
        int scan = 1;
        for (int c = 0; c < images.count(); c++)
        {
            for (int i = 0; i < images[c].count(); i++)
            {
                int id = classes[c][i];
                Matrix in = ica.pca.project(images[c][i]);
                Matrix out = ica.ica.project(in);

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

    Matrix image = test->ica.backProject(test->parameters);
    image = MatrixConverter::columnVectorToMatrix(image, test->imgWidth);
    cv::Size size(image.rows*4, image.cols*4);
    cv::resize(image, image, size);
    cv::imshow(test->winname, image);
}
