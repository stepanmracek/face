#ifndef EVALUATE3DFRGC2_H
#define EVALUATE3DFRGC2_H

#include "linalg/metrics.h"
#include "linalg/gabor.h"
#include "linalg/gausslaguerre.h"
#include "linalg/loader.h"
#include "biometrics/filterfeatureextractor.h"
#include "biometrics/evaluation.h"
#include "biometrics/scorelevelfusionwrapper.h"
#include "biometrics/scorelevefusion.h"
#include "biometrics/biodataprocessing.h"
#include "biometrics/zpcacorrw.h"
#include "biometrics/multiextractor.h"
#include "biometrics/multitemplate.h"
#include "biometrics/multibiomertricsautotuner.h"
#include "linalg/differenceofgaussians.h"

class Evaluate3dFrgc2
{
private:
    static void autoTuner()
    {
        Face::Biometrics::MultiBiomertricsAutoTuner::Input frgcData =
                Face::Biometrics::MultiBiomertricsAutoTuner::Input::fromDirectoryWithExportedCurvatureImages("/home/stepo/data/frgc/spring2004/zbin-aligned2/", "d", 300);

        Face::Biometrics::MultiBiomertricsAutoTuner::Input kinectData =
                Face::Biometrics::MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes("../../test/kinect/", "-");

        Face::Biometrics::MultiBiomertricsAutoTuner::Settings settings(Face::Biometrics::MultiBiomertricsAutoTuner::Settings::FCT_SVM, "allUnits");

        Face::Biometrics::MultiExtractor extractor = Face::Biometrics::MultiBiomertricsAutoTuner::train(frgcData, kinectData, settings);
        extractor.serialize("kinect");
    }

    static int ksize1;
    static int ksize2;
    static bool equalizeDoG;

private:

    static Face::LinAlg::Vector dog(const Matrix &in)
    {
        return Face::LinAlg::MatrixConverter::matrixToColumnVector(
                    Face::LinAlg::MatrixConverter::scale(
                        Face::LinAlg::DifferenceOfGaussians::dog(in, ksize1, ksize2, equalizeDoG), 0.5));
    }

    static Face::LinAlg::Vector equalize(const Matrix &in)
    {
        return Face::LinAlg::MatrixConverter::matrixToColumnVector(
                    Face::LinAlg::MatrixConverter::scale(
                        Face::LinAlg::MatrixConverter::equalize(in), 0.5));
    }

    static Face::LinAlg::Vector plain(const Matrix &in)
    {
        return Face::LinAlg::MatrixConverter::matrixToColumnVector(Face::LinAlg::MatrixConverter::scale(in, 0.5));
    }

    static QVector<Face::LinAlg::Vector> process(const QVector<Matrix> &in, Face::LinAlg::Vector f(const Matrix &))
    {
        QVector<Face::LinAlg::Vector> result;
        foreach (const Matrix &m, in)
        {
            result << f(m);
        }
        return result;
    }

    static double evaluate(const QList<QVector<Matrix> > &imagesInClusters,
                           const QList<QVector<int> > &classesInClusters,
                           Face::LinAlg::Vector f(const Matrix &))
    {
        int N = imagesInClusters.count();

        // train
        QVector<Face::LinAlg::Vector> trainVectors = process(imagesInClusters[0], f);
        Face::Biometrics::ZPCACorrW pca(trainVectors, 0.995, trainVectors);

        QVector<double> eers;
        // evaluate
        for (int i = 1; i < N; i++)
        {
            QVector<Face::LinAlg::Vector> testVectors = process(imagesInClusters[i], f);
            Face::Biometrics::Evaluation e(testVectors, classesInClusters[i], pca.extractor, pca.metric);
            eers << e.eer;
        }

        return Face::LinAlg::Vector(eers).meanValue();
    }

public:

    static void evaluateDoGTexture()
    {
        QString dir("/home/stepo/data/frgc/spring2004/zbin-aligned2/");

        qDebug() << "loading textureImages";
        QVector<Matrix> textureImages;
        QVector<int> ids;
        Face::LinAlg::Loader::loadMatrices(dir + "textureI", textureImages, ids, "d", "*.gz");

        qDebug() << "dividing";
        QList<QVector<Matrix> > imagesInClusters;
        QList<QVector<int> > classesInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters(textureImages, ids, 5, imagesInClusters, classesInClusters);

        //qDebug() << "plain" << evaluate(imagesInClusters, classesInClusters, plain);

        //qDebug() << "equalized" << evaluate(imagesInClusters, classesInClusters, equalize);

        QVector<int> ksizes;
        //ksizes << 3 << 5 << 7 << 9 << 11 << 13 << 15 << 17 << 19 << 21 << 23 << 25 << 27 << 29 << 31 << 33;
        ksizes << 35 << 37 << 39 << 41 << 43 << 45 << 47 << 49 << 51 << 53 << 55;
        for (int e = 0; e <= 1; e++)
        {
            equalizeDoG = e;
            for (int i1 = 0; i1 < ksizes.count() - 1; i1++)
            {
                ksize1 = ksizes[i1];
                for (int i2 = i1 + 1; i2 < ksizes.count(); i2++)
                {
                    ksize2 = ksizes[i2];
                    qDebug() << "dog" << ksize1 << ksize2 << equalizeDoG << evaluate(imagesInClusters, classesInClusters, dog);
                }
            }
        }
    }
};

#endif // EVALUATE3DFRGC2_H
