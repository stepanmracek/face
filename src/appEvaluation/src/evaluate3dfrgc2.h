#ifndef EVALUATE3DFRGC2_H
#define EVALUATE3DFRGC2_H

#include "faceCommon/linalg/metrics.h"
#include "faceCommon/linalg/gabor.h"
#include "faceCommon/linalg/gausslaguerre.h"
#include "faceCommon/linalg/loader.h"
#include "faceCommon/linalg/differenceofgaussians.h"
#include "faceCommon/biometrics/filterfeatureextractor.h"
#include "faceCommon/biometrics/evaluation.h"
#include "faceCommon/biometrics/scorelevelfusionwrapper.h"
#include "faceCommon/biometrics/scorelevefusion.h"
#include "faceCommon/biometrics/biodataprocessing.h"
#include "faceCommon/biometrics/zpcacorrw.h"
#include "faceCommon/biometrics/multiextractor.h"
#include "faceCommon/biometrics/multitemplate.h"
#include "faceCommon/biometrics/multibiomertricsautotuner.h"

using namespace Face::Biometrics;
using namespace Face::LinAlg;

class Evaluate3dFrgc2
{
private:
    static void autoTuner()
    {
        MultiBiomertricsAutoTuner::Input frgcData =
                MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(
                    "/home/stepo/data/frgc/spring2004/zbin-aligned2/", "d", 300);

        MultiBiomertricsAutoTuner::Input kinectData =
                MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(
                    "../../test/kinect/", "-");

        MultiBiomertricsAutoTuner::Settings settings(ScoreSVMFusion::name(),
                                                                       "allUnits");

        MultiExtractor::Ptr extractor = MultiBiomertricsAutoTuner::trainWithWrapper(
                    frgcData, kinectData, settings);

        extractor->serialize("kinect");
    }

    static int ksize1;
    static int ksize2;
    static bool equalizeDoG;

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

    static std::vector<Face::LinAlg::Vector> process(const std::vector<Matrix> &in, Face::LinAlg::Vector f(const Matrix &))
    {
        std::vector<Face::LinAlg::Vector> result;
        foreach (const Matrix &m, in)
        {
            result << f(m);
        }
        return result;
    }

    static double createTemplates(const QList<std::vector<Matrix> > &imagesInClusters,
                           const QList<std::vector<int> > &classesInClusters,
                           Face::LinAlg::Vector f(const Matrix &))
    {
        int N = imagesInClusters.count();

        // train
        std::vector<Face::LinAlg::Vector> trainVectors = process(imagesInClusters[0], f);
        ZPCACorrW pca(trainVectors, 0.995, trainVectors);

        std::vector<double> eers;
        // evaluate
        for (int i = 1; i < N; i++)
        {
            std::vector<Face::LinAlg::Vector> testVectors = process(imagesInClusters[i], f);
            Evaluation e(testVectors, classesInClusters[i], pca.extractor, pca.metric);
            eers << e.eer;
        }

        return Face::LinAlg::Vector(eers).meanValue();
    }

public:

    static void evaluateDoGTexture()
    {
        QString dir("/home/stepo/data/frgc/spring2004/zbin-aligned2/");

        qDebug() << "loading textureImages";
        std::vector<Matrix> textureImages;
        std::vector<int> ids;
        Face::LinAlg::Loader::loadMatrices(dir + "textureI", textureImages, ids, "d", "*.gz");

        qDebug() << "dividing";
        QList<std::vector<Matrix> > imagesInClusters;
        QList<std::vector<int> > classesInClusters;
        BioDataProcessing::divideToNClusters(textureImages, ids, 5, imagesInClusters, classesInClusters);

        //qDebug() << "plain" << evaluate(imagesInClusters, classesInClusters, plain);

        //qDebug() << "equalized" << evaluate(imagesInClusters, classesInClusters, equalize);

        std::vector<int> ksizes;
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
                    qDebug() << "dog" << ksize1 << ksize2 << equalizeDoG << createTemplates(imagesInClusters, classesInClusters, dog);
                }
            }
        }
    }

    static void createMultiExtractor()
    {
        QString unitsFile = "../../test/allUnits";
        QString frgcDirectory = "/home/stepo/data/frgc/spring2004/zbin-aligned2/";

        qDebug() << "loading training set";
        MultiBiomertricsAutoTuner::Input trainData =
                MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(frgcDirectory, "d", 416, 0);

        qDebug() << "loading validation set";
        MultiBiomertricsAutoTuner::Input validationData =
                MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(frgcDirectory, "d", 451, 416);

        MultiBiomertricsAutoTuner::Settings settings(ScoreSVMFusion::name(), unitsFile);
        MultiExtractor::Ptr extractor = MultiBiomertricsAutoTuner::trainWithWrapper(trainData, validationData, settings);

        //eval.outputResults("frgc-SVM", 50);
        extractor->serialize("out");
    }

    static void createTemplates()
    {
        QString frgcDirectory = "/home/stepo/data/frgc/spring2004/zbin-aligned2/";
        QString unitsPath = "/home/stepo/git/tbs-devices/3D_Face/facelib/test/units-test";
        MultiBiomertricsAutoTuner::Settings settings(ScoreWeightedSumFusion::name(), unitsPath);

        auto trainData = MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(frgcDirectory, "d", 416);
        qDebug() << "Train data loaded:" << trainData.ids.count();

        for (int part = 0; part < 2; part++)
        {
            auto testData = (part == 0) ?
                        MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(frgcDirectory, "d", 1000, 416) :
                        MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(frgcDirectory, "d", -1, 1416);
            qDebug() << "Test data loaded:" << testData.ids.count();
            int pCount = settings.params.count();
            #pragma omp parallel for
            for (int p = 0; p < pCount; p++)
            {
                const QString &params = settings.params[p];
                auto unit = MultiExtractor::Unit::parse(params);
                unit->train(trainData.ids, trainData.imageData);

                QDir().mkdir(QString::number(p));
                int n = testData.ids.count();
                std::vector<Template> templates;
                for (int i = 0; i < n; i++)
                {
                    Template t(testData.ids[i], unit->extract(testData.imageData[i]));
                    t.featureVector.toFile(QString::number(p) + "/" + QString::number(t.subjectID) + "-" + QString::number(i));
                    templates << t;
                }

                qDebug() << unit->writeParams() << Evaluation(templates, *(unit->metrics)).eer;
            }
        }
    }

    static void evaluateTemplates()
    {
        QList<cv::Ptr<Face::LinAlg::Metrics> > metrics;
        metrics << new Face::LinAlg::CorrelationMetric()
                << new Face::LinAlg::CorrelationMetric()
                << new Face::LinAlg::CityblockMetric()
                << new Face::LinAlg::CityblockMetric();
        for (int i = 0; i < 6; i++)
        {
            metrics << new Face::LinAlg::CorrelationMetric()
                    << new Face::LinAlg::EuclideanMetric()
                    << new Face::LinAlg::CityblockMetric();
        }

        QList<Evaluation> testEvals;
        QList<Evaluation> trainEvals;

        std::vector<int> indicies; indicies << 10 << 4 << 1 << 2 << 7;
        //for (int algIndex = 0; algIndex < metrics.count(); algIndex++)
        foreach (int algIndex, indicies)
        {
            Face::LinAlg::Metrics::Ptr metric = metrics[algIndex];
            QDir dir("../../test/templates/" + QString::number(algIndex), QString(), QDir::Name, QDir::Files);
            auto templatePaths = dir.entryInfoList();
            int tCount = templatePaths.count();
            std::vector<Template> trainTemplates, testTemplates;
            for (int tIndex = 0; tIndex < tCount; tIndex++)
            {
                auto vec = Vector::fromFile(templatePaths[tIndex].absoluteFilePath());
                Template t;
                t.subjectID = templatePaths[tIndex].baseName().split("-")[0].toInt();
                t.featureVector = vec;

                if (tIndex < 197)
                    trainTemplates << t;
                else
                    testTemplates << t;
            }

            Evaluation trainEval(trainTemplates, *metric);
            Evaluation testEval(testTemplates, *metric);
            trainEvals << trainEval;
            testEvals << testEval;

            qDebug() << algIndex << metric->writeParams() << trainEval.eer << testEval.eer;
        }

        QStringList fusionNames; fusionNames << ScoreSumFusion::name(); // << ScoreSVMFusion::name();
        QStringList normNames;
        normNames << ScoreNormalizerZScore::name();
                  /*<< ScoreNormalizerPass::name() << ScoreNormalizerMean::name() << ScoreNormalizerMedian::name()
                  << ScoreNormalizerZScore::name() << ScoreNormalizerZScore::name()+"Comp"
                  << ScoreNormalizerTanh::name() << ScoreNormalizerTanh::name()+"Comp"
                  << ScoreNormalizerMAD::name() << ScoreNormalizerMAD::name()+"Comp";*/
        QStringList names;
        foreach (const QString &fname, fusionNames)
        {
            foreach (const QString &nName, normNames)
            {
                names << (fname + "-" + nName);
            }
        }

        foreach (const QString &name, names)
        {
            auto fusionResult = ScoreLevelFusionWrapper::trainClassifier(name, trainEvals, true);

            QList<Evaluation> filteredTestEvals;
            foreach (int i, fusionResult.selectedComponents)
            {
                filteredTestEvals << testEvals[i];
                testEvals[i].outputResults(QString::number(i), 50);
            }

            auto fusionEvaluation = fusionResult.fusion->evaluate(filteredTestEvals);
            qDebug() << name;
            fusionEvaluation.printStats();
            fusionEvaluation.outputResults("fusion", 50);
        }
    }
};

#endif // EVALUATE3DFRGC2_H
