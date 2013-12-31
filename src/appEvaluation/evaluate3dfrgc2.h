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

class Evaluate3dFrgc2
{
private:
    static void addGaborEvaluations(QString type, QList<Evaluation> &evals, const QVector<Matrix> &train1images,
                                    const QVector<int> &train1Ids, const QVector<Matrix> &train2images, const QVector<int> &train2Ids)
    {
        HammingMetric m;

        QVector<double> scales; scales << 0.5; // << 0.25 << 0.2 << 0.1;
        QVector<int> gaborSigmas; gaborSigmas << 1 << 2 << 3 << 4 << 5 << 6 << 7;
        QVector<int> gaborThetas; gaborThetas << 0 << 1 << 2 << 3 << 4 << 5 << 6 << 7;

        foreach(double scale, scales)
        {
            foreach(int sigma, gaborSigmas)
            {
                foreach (int theta, gaborThetas)
                {
                    /*Matrix realKernel, imagKernel;
                    Gabor::createWavelet(realKernel, imagKernel, sigma, theta);
                    FilterFeatureExtractor extractor(realKernel, imagKernel, scale);

                    QVector<Vector> train1Vectors;
                    foreach(const Matrix &train1img, train1images)
                        train1Vectors << extractor.extractAbsoluteResponse(train1img);

                    QVector<Vector> train2Vectors;
                    foreach(const Matrix &train2img, train2images)
                        train2Vectors << extractor.extractAbsoluteResponse(train2img);

                    ZPCACorrW zpca(train1Vectors, 0.995, train1Vectors);

                    Evaluation e(train2Vectors, train2Ids, zpca.extractor, zpca.metric);*/
                    qDebug() << "gabor" << type.toStdString().c_str() << scale << sigma << theta;
                    //evals << e;
                }
            }
        }
    }

    static void addGaussLaguerreEvaluations(QString type, QList<Evaluation> &evals, const QVector<Matrix> &train1images,
                                            const QVector<int> &train1Ids, const QVector<Matrix> &train2images, const QVector<int> &train2Ids)
    {
        HammingMetric m;

        QVector<double> scales; scales << 0.5; // << 0.25 << 0.2 << 0.1;
        QVector<int> glSizes; glSizes << 16 << 24 << 32 << 48 << 64 << 72 << 96;
        QVector<int> glNs; glNs << 1 << 2 << 3 << 4 << 5;

        foreach(double scale, scales)
        {
            foreach(int glSize, glSizes)
            {
                foreach (int glN, glNs)
                {
                    /*Matrix realKernel, imagKernel;
                    GaussLaguerre::createWavelet(realKernel, imagKernel, glSize, glN, 0);
                    FilterFeatureExtractor extractor(realKernel, imagKernel, scale);

                    QVector<Vector> train1Vectors;
                    foreach(const Matrix &train1img, train1images)
                        train1Vectors << extractor.extractAbsoluteResponse(train1img);

                    QVector<Vector> train2Vectors;
                    foreach(const Matrix &train2img, train2images)
                        train2Vectors << extractor.extractAbsoluteResponse(train2img);

                    ZPCACorrW zpca(train1Vectors, 0.995, train1Vectors);

                    Evaluation e(train2Vectors, train2Ids, zpca.extractor, zpca.metric);*/
                    qDebug() << "gl" << type.toStdString().c_str() << scale << glSize << glN;
                    //evals << e;
                }
            }
        }
    }

    static void addPlainEvaluations(QString type, QList<Evaluation> &evals, const QVector<Matrix> &train1images,
                                    const QVector<int> &train1Ids, const QVector<Matrix> &train2images, const QVector<int> &train2Ids)
    {
        double scale = 0.5;
        /*Matrix realKernel = Matrix::ones(1, 1);
        Matrix imagKernel = Matrix::ones(1, 1);
        FilterFeatureExtractor extractor(realKernel, imagKernel, scale);

        QVector<Vector> train1Vectors;
        foreach(const Matrix &train1img, train1images)
            train1Vectors << extractor.extractAbsoluteResponse(train1img);

        QVector<Vector> train2Vectors;
        foreach(const Matrix &train2img, train2images)
            train2Vectors << extractor.extractAbsoluteResponse(train2img);

        ZPCACorrW zpca(train1Vectors, 0.995, train1Vectors);

        Evaluation e(train2Vectors, train2Ids, zpca.extractor, zpca.metric);*/
        qDebug() << "plain" << type.toStdString().c_str() << scale;
        //evals << e;
    }

public:
    static void evaluateFilterResponse()
    {
        int clusterCount = 5;
        QStringList types; types << "depth" << "index" << "mean" << "gauss" << "eigencur" << "textureE";
        QString dir = "/home/stepo/data/frgc/spring2004/zbin-aligned2/";

        QVector<int> ids;
        QVector<QString> files = Loader::listFiles(dir + types[0] + "/", "*.gz", Loader::BaseFilename);
        foreach (const QString &file, files)
        {
            ids << file.split('d').at(0).toInt();
        }

        QList<Evaluation> evals;
        foreach(QString type, types)
        {
            /*QVector<Matrix> images;
            foreach (const QString &file, files)
            {
                images << Common::loadMatrix(dir + type + "/" + file + ".gz");
            }*/

            QList<QVector<Matrix> > imagesInClusters;
            QList<QVector<int> > idsInClusters;
            //BioDataProcessing::divideToNClusters(images, ids, clusterCount, imagesInClusters, idsInClusters);

            addPlainEvaluations(type, evals, imagesInClusters[0], idsInClusters[0], imagesInClusters[1], idsInClusters[1]);
            addGaborEvaluations(type, evals, imagesInClusters[0], idsInClusters[0], imagesInClusters[1], idsInClusters[1]);
            addGaussLaguerreEvaluations(type, evals, imagesInClusters[0], idsInClusters[0], imagesInClusters[1], idsInClusters[1]);
        }

        //for (int i = 0; i < evals.count(); i++)
        //{
        //    Vector(evals[i].genuineScores).toFile("gen-" + QString::number(i));
        //    Vector(evals[i].impostorScores).toFile("imp-" + QString::number(i));
        //}
    }

    static void learnFilterResponseFusion()
    {
        QVector<QString> impFiles = Loader::listFiles(".", "imp-*", Loader::Filename);
        QVector<QString> genFiles = Loader::listFiles(".", "gen-*", Loader::Filename);

        QList<Evaluation> evals;
        for (int i = 0; i < impFiles.count(); i++)
        {
            Vector gen = Vector::fromFile(genFiles[i]);
            Vector imp = Vector::fromFile(impFiles[i]);

            int num = impFiles[i].split('-').at(1).toInt();
            Evaluation e(gen.toQVector(), imp.toQVector());
            qDebug() << num << genFiles[i] << impFiles[i] << e.eer;

            evals << e;
        }

        ScoreSVMFusion fusion;
        ScoreLevelFusionWrapper wrapper;
        wrapper.trainClassifier(fusion, evals, true);
    }

    static MultiExtractor::UnitFilter *trainPlain(const QString &type, double scale,
                                                  QMap<QString, QList<QVector<Matrix> > > &imagesInClusters,
                                                  QList<QVector<int> > &classesInClusters,
                                                  QList<Evaluation> &evaluations)
    {
        qDebug() << "training plain" << type;
        MultiExtractor::UnitFilter *u = MultiExtractor::UnitFilter::train(type, scale, imagesInClusters[type][0], imagesInClusters[type][0]);
        evaluations << Evaluation(u->filter.extractAbsoluteResponse(imagesInClusters[type][1]), classesInClusters[1], u->pca, CorrelationMetric());
        return u;
    }

    static MultiExtractor::UnitGabor *trainGabor(const QString &type, double scale, int freq, int orientation,
                                                 QMap<QString, QList<QVector<Matrix> > > &imagesInClusters,
                                                 QList<QVector<int> > &classesInClusters,
                                                 QList<Evaluation> &evaluations)
    {
        qDebug() << "training gabor" << type << freq << orientation;
        MultiExtractor::UnitGabor *u = MultiExtractor::UnitGabor::train(type, freq, orientation, scale, imagesInClusters[type][0], imagesInClusters[type][0]);
        evaluations << Evaluation(u->filter.extractAbsoluteResponse(imagesInClusters[type][1]), classesInClusters[1], u->pca, CorrelationMetric());
        return u;
    }

    static MultiExtractor::UnitGaussLaguerre *trainGaussLaguerre(const QString &type, double scale, int size, int n,
                                                                 QMap<QString, QList<QVector<Matrix> > > &imagesInClusters,
                                                                 QList<QVector<int> > &classesInClusters,
                                                                 QList<Evaluation> &evaluations)
    {
        qDebug() << "training gl" << type << size << n;
        MultiExtractor::UnitGaussLaguerre *u = MultiExtractor::UnitGaussLaguerre::train(type, size, n, scale, imagesInClusters[type][0], imagesInClusters[type][0]);
        evaluations << Evaluation(u->filter.extractAbsoluteResponse(imagesInClusters[type][1]), classesInClusters[1], u->pca, CorrelationMetric());
        return u;
    }

    static MultiExtractor::Unit *train(const QString lineParams,
                                       QMap<QString, QList<QVector<Matrix> > > &imagesInClusters,
                                       QList<QVector<int> > &classesInClusters,
                                       QList<Evaluation> &evaluations)
    {
        if (lineParams.isEmpty() || lineParams.isNull()) return 0;

        QStringList params = lineParams.split(QRegExp("\\s+"));

        QString type = params.at(1);
        double scale = params.at(2).toDouble();

        if (params.at(0).compare("plain") == 0)
        {
            return trainPlain(type, scale, imagesInClusters, classesInClusters, evaluations);
        }
        else if (params.at(0).compare("gabor") == 0)
        {
            int frequency = params.at(3).toInt();
            int orientation = params.at(4).toInt();

            return trainGabor(type, scale, frequency, orientation, imagesInClusters, classesInClusters, evaluations);
        }
        else if (params.at(0).compare("gl") == 0)
        {
            int size = params.at(3).toInt();
            int n = params.at(4).toInt();

            return trainGaussLaguerre(type, scale, size, n, imagesInClusters, classesInClusters, evaluations);
        }
        return 0;
    }

    static void load(QMap<QString, QList<QVector<Matrix> > > &imagesInClusters,
                     QList<QVector<int> > &idsInClusters,
                     int imageCount, int clusterCount)
    {
        QStringList types; types << "depth" << "index" << "mean" << "gauss" << "eigencur" << "textureE";
        QString dir = "/home/stepo/data/frgc/spring2004/zbin-aligned2/";

        QVector<int> ids;
        QVector<QString> files = Loader::listFiles(dir + types[0] + "/", "*.gz", Loader::BaseFilename);

        if (imageCount > 0 && imageCount <= files.count())
            files.resize(imageCount);
        foreach (const QString &file, files) ids << file.split('d').at(0).toInt();

        foreach(QString type, types)
        {
            qDebug() << "loading" << type;
            imagesInClusters[type] = QList<QVector<Matrix> >();

            QVector<Matrix> images;
            foreach (const QString &file, files)
            {
                images << Common::loadMatrix(dir + type + "/" + file + ".gz");
            }

            idsInClusters.clear();
            BioDataProcessing::divideToNClusters(images, ids, clusterCount, imagesInClusters[type], idsInClusters);
        }
    }

    static QStringList loadUnitsFile(const QString &path)
    {
        QStringList result;
        QFile f(path);
        f.open(QIODevice::ReadOnly);
        QTextStream in(&f);

        while (!in.atEnd())
        {
            QString line = in.readLine();
            if (line.isNull() || line.isEmpty()) continue;
            result << line;
        }

        return result;
    }

    static void serializeExtractor()
    {
        int clusterCount = 2;
        int imageCount = 867;
        QVector<int> selectedUnits;
        //selectedUnits << 72 << 127 << 496 << 209 << 286 << 498 << 121 << 397; //wsum
        selectedUnits << 72 << 209 << 490 << 496 << 127 << 310 << 450 << 168 << 174
                      << 234 << 156 << 453 << 386 << 548 << 162 << 268 << 200 << 539 << 171; //svm
        QStringList allUnits = loadUnitsFile("allUnits");

        QMap<QString, QList<QVector<Matrix> > > imagesInClusters;
        QList<QVector<int> > idsInClusters;
        load(imagesInClusters, idsInClusters, imageCount, clusterCount);

        MultiExtractor extractor;
        QList<Evaluation> evals;
        foreach(int unitIndex, selectedUnits)
        {
            extractor.units << train(allUnits.at(unitIndex), imagesInClusters, idsInClusters, evals);
        }

        qDebug() << "training fusion";

        extractor.fusionType = "SVM"; //"weightedSum";
        extractor.fusion = new ScoreSVMFusion(); // new ScoreWeightedSumFusion();
        foreach(const Evaluation &e, evals)
            extractor.fusion->addComponent(e);
        extractor.fusion->learn();

        qDebug() << "saving";
        extractor.serialize("svm"); //wsum");
        qDebug() << "OK";
    }

    static void deserialize()
    {
        QStringList types; types << "depth" << "index" << "mean" << "gauss" << "eigencur" << "textureE";
        MultiExtractor extractor("svm"); // "wsum");
        int clusterCount = 5;
        int imageCount = -1;
        QMap<QString, QList<QVector<Matrix> > > imagesInClusters;
        QList<QVector<int> > idsInClusters;
        load(imagesInClusters, idsInClusters, imageCount, clusterCount);

        for (int cluster = 1; cluster < clusterCount; cluster++)
        {
            qDebug() << "creating templates";
            QVector<MultiTemplate> templates;

            for (int i = 0; i < idsInClusters[cluster].count(); i++)
            {
                MultiExtractor::ImageData data;
                foreach(const QString &type, types)
                    data[type] = imagesInClusters[type][cluster][i];

                templates << extractor.extract(data, 0, idsInClusters[cluster][i]);
            }

            qDebug() << "evaluating";
            Evaluation e = extractor.evaluate(templates);
            qDebug() << e.eer;
        }
    }

    static void autoTuner()
    {
        MultiBiomertricsAutoTuner::Input frgcData =
                MultiBiomertricsAutoTuner::Input::fromDirectoryWithExportedCurvatureImages("/home/stepo/data/frgc/spring2004/zbin-aligned2/", "d", 300);

        MultiBiomertricsAutoTuner::Input kinectData =
                MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedBinMeshes("../../test/kinect/", "-");

        MultiBiomertricsAutoTuner::Settings settings(MultiBiomertricsAutoTuner::FCT_SVM, "allUnits");

        MultiExtractor extractor = MultiBiomertricsAutoTuner::train(frgcData, kinectData, settings);
        extractor.serialize("kinect");
    }
};

#endif // EVALUATE3DFRGC2_H
