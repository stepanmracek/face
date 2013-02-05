#ifndef TESTPCA_H
#define TESTPCA_H

#include <QVector>
#include <QDir>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "linalg/vector.h"
#include "linalg/pca.h"
#include "linalg/procrustes.h"
#include "linalg/common.h"
#include "linalg/loader.h"

void onChange(int newVal, void *userData);

class TestPCA
{
public:
    static void testPCASimple()
    {
        // load vectors
        QVector<Matrix> vectors;
        QDir dir("vectorsToAlign");
        dir.setFilter(QDir::Files | QDir::NoDotAndDotDot);
        QStringList filenames = dir.entryList();

        for (int i = 0; i < filenames.count(); i++)
        {
            Matrix v = Vector::fromTwoColsFile("vectorsToAlign/" + filenames[i]);
            vectors.append(v);
        }

        // align them
        Procrustes::procrustesAnalysis(vectors);

        PCA pca(vectors);
        for (int i = 0; i < vectors.count(); i++)
        {
            Matrix projected = pca.project(vectors[i]);
            Matrix backProjected = pca.backProject(projected);
            Matrix diff = vectors[i] - backProjected;
            qDebug() << i<< Vector::magnitude(diff);
        }
    }

    static void testPCAStorage()
    {
        // load vectors
        QVector<Matrix> vectors;
        QDir dir("vectorsToAlign");
        dir.setFilter(QDir::Files | QDir::NoDotAndDotDot);
        QStringList filenames = dir.entryList();

        for (int i = 0; i < filenames.count(); i++)
        {
            Matrix v = Vector::fromTwoColsFile("vectorsToAlign/" + filenames[i]);
            vectors.append(v);
        }

        // align them
        Procrustes::procrustesAnalysis(vectors);

        PCA pca(vectors);
        pca.serialize("myPCA.yml");

        PCA pca2("myPCA.yml");
        for (int i = 0; i < vectors.count(); i++)
        {
            Matrix projected = pca2.project(vectors[i]);
            Matrix backProjected = pca2.backProject(projected);
            Matrix diff = vectors[i] - backProjected;
            qDebug() << i<< Vector::magnitude(diff);
        }
    }

    char * winname;
    int trackbarValues[10];
    Matrix parameters;
    PCA pca;
    int imWidth;

    void testPCACreateFaceSpace()
    {
        QVector<Matrix> input;
        //QString path("/home/stepo/SVN/disp-stepan-mracek/databases/orl");
        QString path("/media/frgc/frgc-norm-iterative/big-index");
        Loader::loadImages(path, input, 0, "*.png", true);
        QVector<Matrix> reduced = input.mid(0,196);

        pca.learn(reduced);
        pca.modesSelectionThreshold();
        qDebug() << "modes:" << pca.getModes();

        imWidth = 121;
        parameters = Matrix::zeros(pca.getModes(), 1);
        Matrix image = pca.backProject(parameters);
        image = MatrixConverter::columnVectorToMatrix(image, imWidth);

        winname = "PCA FaceSpace";
        cv::namedWindow(winname);
        for (int i = 0; i < 10; i++)
        {
            trackbarValues[i] = 5;
            std::string tbname = QString::number(i).toStdString();
            cv::createTrackbar(tbname, winname, &(trackbarValues[i]), 10, onChange, this);
        }

        cv::imshow(winname, image);
        cv::waitKey();
    }
};

void onChange(int, void* userData)
{
    TestPCA *test = (TestPCA*)userData;
    for (int i = 0; i < 10; i++)
    {
        std::string tbname = QString::number(i).toStdString();
        double value = cv::getTrackbarPos(tbname, test->winname);
        value = (value-5.0)/5.0 * (3*sqrt(test->pca.cvPca.eigenvalues.at<double>(i)));
        test->parameters(i) = value;
    }

    Matrix image = test->pca.backProject(test->parameters);
    image = MatrixConverter::columnVectorToMatrix(image, test->imWidth);
    cv::imshow(test->winname, image);
}

#endif // TESTPCA_H
