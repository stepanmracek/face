#ifndef EVALUATE3DFRGC_H_
#define EVALUATE3DFRGC_H_

#include <QVector>
#include <QString>
#include <QDebug>
#include <QSet>
#include <cassert>

#include "linalg/common.h"
#include "linalg/loader.h"
#include "linalg/vector.h"
#include "linalg/pca.h"
#include "linalg/matrixconverter.h"
#include "biometrics/biodataprocessing.h"
#include "biometrics/featureextractor.h"
#include "biometrics/discriminativepotential.h"
#include "biometrics/scorelevefusion.h"
#include "biometrics/filterfeatureextractor.h"
#include "facedata/mesh.h"
#include "facedata/surfaceprocessor.h"
#include "facedata/landmarkdetector.h"
#include "facedata/landmarks.h"
#include "facedata/facealigner.h"
#include "facedata/glwidget.h"
#include "linalg/kernelgenerator.h"
#include "linalg/serialization.h"
#include "biometrics/isocurveprocessing.h"
#include "linalg/histogram.h"
#include "biometrics/histogramfeatures.h"
#include "biometrics/geneticweightoptimization.h"
#include "biometrics/eerpotential.h"
#include "linalg/adaboost.h"
#include "linalg/gausslaguerre.h"
#include "linalg/gabor.h"
#include "biometrics/scorelevelfusionwrapper.h"
#include "biometrics/zpcacorrw.h"
#include "biometrics/facetemplate.h"

class Evaluate3dFrgc
{
public:
	static void compare(QVector<int> &first, QVector<int> &second)
	{
		int n = first.count();
		assert(n == second.count());
		for (int i = 0; i < n; i++)
			assert(first[i] == second[i]);
	}

    static void align()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/bin/";
        QString outDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned2/";

        Face::FaceData::FaceAligner aligner(Face::FaceData::Mesh::fromOBJ("../../test/meanForAlign.obj"));
        Face::FaceData::MapConverter converter;

        QDir srcDir(srcDirPath, "*.bin");
        QFileInfoList srcFiles = srcDir.entryInfoList();
        foreach (const QFileInfo &srcFileInfo, srcFiles)
        {
            Face::FaceData::Mesh mesh = Face::FaceData::Mesh::fromBIN(srcFileInfo.absoluteFilePath());
            aligner.icpAlign(mesh, 15, Face::FaceData::FaceAligner::NoseTipDetection);

            QString resultPath = outDirPath + srcFileInfo.baseName() + ".binz";
            mesh.writeBINZ(resultPath);

            Face::FaceData::Map texture =
                    Face::FaceData::SurfaceProcessor::depthmap(mesh, converter, cv::Point2d(-100,-100), cv::Point2d(100,100),
                                                               1, Face::FaceData::SurfaceProcessor::Texture_I);
            Matrix m = texture.toMatrix(0, 0, 255);
            cv::circle(m, cv::Point(100,100), 3, 255, -1);
            QString resultTexturePath = outDirPath + srcFileInfo.baseName() + ".png";
            cv::imwrite(resultTexturePath.toStdString(), m*255);
            cv::imshow("test", m);
            cv::waitKey(1);
        }
    }

    static void createIsoCurves()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned2/";
        QString outDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned2/isocurves2/";
        QDir srcDir(srcDirPath, "*.binz");
        QFileInfoList srcFiles = srcDir.entryInfoList();
        foreach (const QFileInfo &srcFileInfo, srcFiles)
        {
            QString resultPath = outDirPath + srcFileInfo.baseName() + ".xml";
            if (QFile::exists(resultPath)) continue;

            QVector<Face::FaceData::VectorOfPoints> isoCurves;
            Face::FaceData::Mesh mesh = Face::FaceData::Mesh::fromBINZ(srcFileInfo.absoluteFilePath());
            isoCurves = Face::Biometrics::Face3DTemplate::getIsoGeodesicCurves(mesh);
            Face::LinAlg::Serialization::serializeVectorOfPointclouds(isoCurves, resultPath);
            //LandmarkDetector detector(mesh);
            //Landmarks lm = detector.detect();
            //cv::Point3d nosetip = lm.get(Landmarks::Nosetip);
            //mesh.translate(-nosetip);

            /*MapConverter converter;
            Map depth = SurfaceProcessor::depthmap(mesh, converter, 2, ZCoord);
            Matrix gaussKernel = KernelGenerator::gaussianKernel(7);
            depth.applyFilter(gaussKernel, 3, true);

            int startD = 10;
            for (int d = startD; d <= 100; d += 10)
            {
                VectorOfPoints isoCurve = SurfaceProcessor::isoGeodeticCurve(depth, converter, cv::Point3d(0,20,0), d, 100, 2);
                isoCurves << isoCurve;
            }*/
        }
    }

    static void evaluateIsoCurves()
    {
        //int modulo = 1;
        QString dirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned2/isocurves/";
        QVector<Face::Biometrics::SubjectIsoCurves> data = Face::Biometrics::IsoCurveProcessing::readDirectory(dirPath, "d", "*.xml");
        Face::Biometrics::IsoCurveProcessing::selectIsoCurves(data, 0, 5);
        Face::Biometrics::IsoCurveProcessing::stats(data);
        //IsoCurveProcessing::sampleIsoCurvePoints(data, modulo);
        QVector<Face::Biometrics::Template> rawData = Face::Biometrics::IsoCurveProcessing::generateTemplates(data, false);

        QVector<int> classes;
        QVector<Face::LinAlg::Vector> rawFeatureVectors;
        Face::Biometrics::Template::splitVectorsAndClasses(rawData, rawFeatureVectors, classes);

        int clusterCount = 5;
        QList<QVector<int> > classesInClusters;
        QList<QVector<Face::LinAlg::Vector> > rawVectorsInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters(rawFeatureVectors, classes, clusterCount,
                                                               rawVectorsInClusters, classesInClusters);

        QList<QVector<int> > trainClassesInClusters;
        QList<QVector<Face::LinAlg::Vector> > trainVectorsInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters(rawVectorsInClusters[0], classesInClusters[0], 2,
                trainVectorsInClusters, trainClassesInClusters);

        Face::LinAlg::PCA pca(trainVectorsInClusters[0]);
        //pca.modesSelectionThreshold(pcaThreshold);
        //PCAExtractor pcaExtractor(pca);
        Face::Biometrics::ZScorePCAExtractor zscorePcaExtractor(pca, trainVectorsInClusters[1]);
        zscorePcaExtractor.serialize("isocurves-pca", "isocurves-normparams");

        QVector<Face::Biometrics::Template> trainTemplates =
                Face::Biometrics::Template::createTemplates(trainVectorsInClusters[1], trainClassesInClusters[1], zscorePcaExtractor);
        Face::Biometrics::EERPotential eerPot(trainTemplates);

        Face::LinAlg::CorrelationWeightedMetric corW;
        double bestThreshold;
        double bestEER = 1;
        for (double selThreshold = 0.0; selThreshold <= 0.3; selThreshold += 0.01)
        {
            corW.w = eerPot.createSelectionWeightsBasedOnRelativeThreshold(selThreshold);
            Face::Biometrics::Evaluation eCor(rawVectorsInClusters[1], classesInClusters[1], zscorePcaExtractor, corW);

            if (eCor.eer < bestEER)
            {
                bestEER = eCor.eer;
                bestThreshold = selThreshold;
            }

            qDebug() << "selThreshold" << selThreshold << eCor.eer;
        }

        corW.w = eerPot.createSelectionWeightsBasedOnRelativeThreshold(bestThreshold);
        corW.w.toFile("isocurves-selWeights");
        Face::Biometrics::BatchEvaluationResult batchResult =
                Face::Biometrics::Evaluation::batch(rawVectorsInClusters, classesInClusters, zscorePcaExtractor, corW, 1);
        qDebug() << batchResult.meanEER << batchResult.stdDevOfEER;
        int i = 0;
        foreach(const Face::Biometrics::Evaluation &e, batchResult.results)
        {
            qDebug() << e.eer;
            e.outputResults("isocurves-" + QString::number(i++), 50);
        }
    }

    static void createTextures()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned2/";
        QDir srcDir(srcDirPath, "*.binz");
        QFileInfoList srcFiles = srcDir.entryInfoList();

        QStringList nans;

        QStringList curvatureNames;
        curvatureNames << "depth" << "mean" << "gauss" << "index" << "eigencur";
        foreach (const QFileInfo &srcFileInfo, srcFiles)
        {
            Face::FaceData::Mesh mesh = Face::FaceData::Mesh::fromBINZ(srcFileInfo.absoluteFilePath(), false);
            /*Matrix equalized = Face3DTemplate::getTexture(mesh, true);

            QString out = srcDirPath + "textureE/" + srcFileInfo.baseName() + ".gz";
            if (Common::matrixContainsNan(equalized))
                nans << "depth-" + srcFileInfo.baseName();
            Common::saveMatrix(equalized, out);

            QList<Matrix> curvatures = Face3DTemplate::getDeMeGaInEi(mesh);
            int index = 0;
            foreach (const QString &curvatureName, curvatureNames)
            {
                QString out = srcDirPath + curvatureName + "/" + srcFileInfo.baseName() + ".gz";
                if (Common::matrixContainsNan(curvatures[index]))
                    nans << curvatureName + "-" + srcFileInfo.baseName();
                Common::saveMatrix(curvatures[index], out);
                index++;
            }*/

            Matrix texture = Face::Biometrics::Face3DTemplate::getTexture(mesh, false);
            QString out = srcDirPath + "textureI/" + srcFileInfo.baseName() + ".gz";
            if (Face::LinAlg::Common::matrixContainsNan(texture))
                nans << "textureI-" + srcFileInfo.baseName();
            Face::LinAlg::Common::saveMatrix(texture, out);
        }

        qDebug() << nans;
    }

    static void evaluateHistogramFeaturesGenerateStripesBinsMap()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/depth2";
        QDir srcDir(srcDirPath, "*.png");
        QFileInfoList srcFiles = srcDir.entryInfoList();

        qDebug() << "Loading...";
        QVector<ImageGrayscale> allImages;
        QVector<int> allClasses;
        foreach (const QFileInfo &fileInfo, srcFiles)
        {
            ImageGrayscale full = cv::imread(fileInfo.absoluteFilePath().toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
            cv::GaussianBlur(full, full, cv::Size(21,21), 0);
            ImageGrayscale cropped = full(cv::Rect(40, 20, 220, 180));
            allImages << cropped;
            allClasses << fileInfo.baseName().split('d')[0].toInt();
        }

        QList<QVector<ImageGrayscale> > imagesInClusters;
        QList<QVector<int> > classesInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters<ImageGrayscale>(allImages, allClasses, 10, imagesInClusters, classesInClusters);

        for (int i = 0; i < 3; i++)
        {
            qDebug() << "Evaluating" << i << imagesInClusters[i].count();
            for (int stripes = 3; stripes <= 50; stripes++)
            {
                for (int bins = 3; bins <= 50; bins++)
                {
                    QVector<Face::LinAlg::Vector> rawVectors;
                    foreach(const ImageGrayscale &image, imagesInClusters[i])
                    {
                        Face::Biometrics::HistogramFeatures features(image, stripes, bins);
                        rawVectors << features.toVector();
                    }

                    Face::Biometrics::Evaluation e(rawVectors, classesInClusters[i], Face::Biometrics::PassExtractor(),
                                                   Face::LinAlg::CityblockMetric());
                    qDebug() << stripes << bins << e.eer; //  results.meanEER << "+-" << results.stdDevOfEER;
                }
                qDebug() << "";
            }
        }
    }

    /*static void evaluateHistogramFeatures()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        QDir srcDir(srcDirPath, "*.binz");
        QFileInfoList srcFiles = srcDir.entryInfoList();

        cv::Point2d meshStart(-60,-30);
        cv::Point2d meshEnd(60, 60);
        int kSize = 9;
        int gaussTimes = 1;
        int stripes = 20;
        int bins = 20;

        //QVector<int> startx; startx << -50 << -50 << -60 << -60 << -60;
        //QVector<int> starty; starty << -30 << -30 << -30 << -40 << -40;
        //QVector<int> endx;   endx   <<  50 <<  50 <<  60 <<  60 <<  60;
        //QVector<int> endy;   endy   <<  50 <<  60 <<  60 <<  60 <<  70;

        //QVector<int> kSizes; kSizes << 3 << 5 << 7 << 9;
        //QVector<int> filterRepeats; filterRepeats << 1 << 2 << 3;

        //QVector<Metrics*> metrics;
        //metrics << new CityblockMetric() << new SumOfSquareDifferences() << new CosineMetric() << new CorrelationMetric();

        MapConverter mapConverter;
        Matrix gaussKernel = KernelGenerator::gaussianKernel(kSize);
        QVector<Map> allMaps;
        QVector<int> allClasses;
        foreach (const QFileInfo &fileInfo, srcFiles)
        {
            Mesh mesh = Mesh::fromBINZ(fileInfo.absoluteFilePath());
            Map depth = SurfaceProcessor::depthmap(mesh, mapConverter, meshStart, meshEnd, 2.0, ZCoord);
            depth.applyFilter(gaussKernel, gaussTimes, true);

            allMaps << depth;
            allClasses << fileInfo.baseName().split('d')[0].toInt();

            if (allClasses.count() == 874) break;
            //Vector histFeatures = HistogramFeatures(depth, stripes, bins).toVector();
            //histFeatures.toFile("/home/stepo/data/frgc/spring2004/zbin-aligned/histogram-20-20/" + fileInfo.baseName());
        }

        QVector<Vector> allVectors;
        foreach(const Map &depth, allMaps)
        {
            Vector histFeatures = HistogramFeatures(depth, stripes, bins).toVector();
            allVectors << histFeatures;
        }

        QList<QVector<int> > classesInClusters;
        QList<QVector<Vector> > vectorsInClusters;
        BioDataProcessing::divideToNClusters<Vector>(allVectors, allClasses, 2, vectorsInClusters, classesInClusters);

        ZScorePassExtractor zPassExtractor(vectorsInClusters[0]);
        zPassExtractor.serialize("../../test/frgc/histogram/normparams.yml");

        Evaluation e(vectorsInClusters[1], classesInClusters[1], zPassExtractor, CorrelationMetric());
        qDebug() << "initial eer" << e.eer;

        QVector<Template> trainTemplates = Template::createTemplates(vectorsInClusters[0], classesInClusters[0], zPassExtractor);
        EERPotential eerPotential(trainTemplates);

        eerPotential.scores.toFile("eerPotentialTrain");

        //CorrelationWeightedMetric corrW;
        //double s = 0.8;
        //Vector selectionWeights = eerPotential.createSelectionWeightsBasedOnRelativeThreshold(s);
        //selectionWeights.toFile("../../test/frgc/histogram/selectionWeights");
        //corrW.w = selectionWeights;

        //Evaluation e2(vectorsInClusters[1], classesInClusters[1], zPassExtractor, corrW);
        //qDebug() << "eerPotential scores" << s << "eer" << e2.eer;
    }*/

    static void evaluateTextures()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        double threshold = 0.995;
        cv::Rect roi(50, 40, 100, 90);

        QVector<Face::LinAlg::Vector> vectors;
        QVector<int> classes;
        Face::LinAlg::Loader::loadImages(srcDirPath, vectors, &classes, "*.png", "d", -1, roi);

        qDebug() << "dividing";
        QList<QVector<Face::LinAlg::Vector> > vectorsInClusters;
        QList<QVector<int> > classesInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters(vectors, classes, 5, vectorsInClusters, classesInClusters);

        qDebug() << "extractor and metric";
        Face::Biometrics::ZPCACorrW extractorAndMetric(vectorsInClusters[0], threshold, vectorsInClusters[0]);//, classesInClusters[1], vectorsInClusters[1], 0.0, 0.5, 0.05);

        qDebug() << "evaluation";
        Face::Biometrics::BatchEvaluationResult batchEval = Face::Biometrics::Evaluation::batch(vectorsInClusters, classesInClusters,
                                                                              extractorAndMetric.extractor, extractorAndMetric.metric);
        foreach(const Face::Biometrics::Evaluation &eval, batchEval.results)
        {
            qDebug() << eval.eer;
        }
    }

    static void evaluateMaps()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        QStringList sources; sources << "depth" << "eigencur" << "gauss"
                                     << "index" << "mean";
        double threshold = 0.995;

        // best: w: 50, u: 60, l: 30
        /*QVector<double> roiWidths; roiWidths << 50 << 50 << 60 << 60 << 60;
        QVector<double> roiUppers; roiUppers << 50 << 60 << 60 << 60 << 70;
        QVector<double> roiLowers; roiLowers << 30 << 30 << 30 << 40 << 40;*/
        //cv::Rect roi2(50, 30, 200, 180);
        cv::Rect roi(25, 15, 100, 90);
        /*cv::Rect roi(150-roiWidths[roiIndex]*2, 150-roiUppers[roiIndex]*2,
                     roiWidths[roiIndex]*4, roiUppers[roiIndex]*2+roiLowers[roiIndex]*2);*/

        //QList<ScoreLevelFusionComponent> components;
        //QList<QVector<Vector> > allData;
        //QVector<int> testClasses;
        foreach (const QString &source, sources)
        {
            QVector<Face::LinAlg::Vector> vectors;
            QVector<int> classes;
            Face::LinAlg::Loader::loadImages(srcDirPath + source, vectors, &classes, "*.png", "d", -1, roi);

            QList<QVector<Face::LinAlg::Vector> > vectorsInClusters;
            QList<QVector<int> > classesInClusters;
            Face::Biometrics::BioDataProcessing::divideToNClusters(vectors, classes, 5, vectorsInClusters, classesInClusters);

            Face::LinAlg::PCA pca(vectorsInClusters[0]);
            pca.modesSelectionThreshold(threshold);
            Face::Biometrics::ZScorePCAExtractor *zPcaExtractor = new Face::Biometrics::ZScorePCAExtractor(pca, vectorsInClusters[0]);
            zPcaExtractor->serialize("../../test/frgc/" + source + "/pca.yml",
                                     "../../test/frgc/" + source + "/normparams.yml");

            Face::LinAlg::Metrics *metrics = new Face::LinAlg::CorrelationMetric();

            Face::Biometrics::Evaluation eval(vectorsInClusters[1], classesInClusters[1], *zPcaExtractor, *metrics);
            qDebug() << source << "z-PCA" << eval.eer;

            //ScoreLevelFusionComponent component(vectorsInClusters[1], classesInClusters[1], zPcaExtractor, metrics);
            //components << component;

            //if (testClasses.count() == 0)
            //    testClasses = classesInClusters[2];
            //allData << vectorsInClusters[2];
        }

        /*ScoreSVMFusion fusion;
        QVector<int> selectedComponents = ScoreLevelFusionWrapper::trainClassifier(fusion, components);
        qDebug() << "Selected components";
        QList<QVector<Vector> > testData;
        foreach (int i, selectedComponents)
        {
            qDebug() << sources[i];
            testData << allData[i];
        }

        qDebug() << "Test EER:" << fusion.evaluate(testData, testClasses).eer;*/
    }

    static void evaluateDirect()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        QString source = "depth2";
        cv::Rect roi(50, 30, 200, 180);

        QVector<Matrix> images;
        QVector<int> classes;
        QVector<Face::LinAlg::Vector> vectors;
        Face::LinAlg::Loader::loadImages(srcDirPath + source, images, &classes, "*.png", "d", 874);
        int n = images.count();

        for (int i = 0; i < n; i++)
        {
            Matrix sub = images[i](roi);
            cv::resize(sub, sub, cv::Size(sub.cols/4, sub.rows/4));
            vectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(sub);
        }

        QList<QVector<Face::LinAlg::Vector> > vectorsInClusters;
        QList<QVector<int> > classesInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters(vectors, classes, 2, vectorsInClusters, classesInClusters);

        Face::Biometrics::ZScorePassExtractor extractor(vectorsInClusters[0]);
        QVector<Face::Biometrics::Template> trainTemplates =
                Face::Biometrics::Template::joinVectorsAndClasses(vectorsInClusters[0], classesInClusters[0]);
        Face::Biometrics::EERPotential potential(trainTemplates);
        Face::LinAlg::CorrelationWeightedMetric metric;
        for (double t = 0.05; t <= 0.95; t += 0.05) // 0.3 je nejlepsi
        {
            metric.w = potential.createSelectionWeightsBasedOnRelativeThreshold(t);
            qDebug() << t << Face::Biometrics::Evaluation(vectorsInClusters[1], classesInClusters[1], extractor, metric).eer;
        }
    }

/*    static void createCurves()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        QString outDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/curves2/";
        QDir srcDir(srcDirPath, "*.binz");
        QFileInfoList srcFiles = srcDir.entryInfoList();
        //QVector<Map> maps;
        foreach (const QFileInfo &srcFileInfo, srcFiles)
        {
            Mesh mesh = Mesh::fromBINZ(srcFileInfo.absoluteFilePath());
            LandmarkDetector detector(mesh);
            Landmarks lm = detector.detect();
            cv::Point3d nosetip = lm.get(Landmarks::Nosetip);
            mesh.translate(-nosetip);

            MapConverter converter;
            Map depth = SurfaceProcessor::depthmap(mesh, converter, cv::Point2d(-65,-45), cv::Point2d(65, 65), 2, ZCoord);
            Matrix gaussKernel = KernelGenerator::gaussianKernel(7);
            depth.applyFilter(gaussKernel, 3, true);

            VectorOfCurves curves;
            for (int x = -60; x <= 60; x += 20)
            {
                curves << SurfaceProcessor::surfaceCurve(depth, converter, cv::Point3d(x, -40, 0), cv::Point3d(x, 60, 0), 100, 2);
            }
            for (int y = -40; y <= 60; y += 20)
            {
                curves << SurfaceProcessor::surfaceCurve(depth, converter, cv::Point3d(-60, y, 0), cv::Point3d(60, y, 0), 100, 2);
            }

            Serialization::serializeVectorOfPointclouds(curves, outDirPath + srcFileInfo.baseName() + ".xml");
        }
    }

    static void evaluateCurves()
    {
        QString dirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/curves2/";
        QVector<SubjectIsoCurves> data = IsoCurveProcessing::readDirectory(dirPath, "d", "*.xml");
        QVector<Template> rawData = IsoCurveProcessing::generateTemplates(data, true);

        QVector<int> classes;
        QVector<Vector> rawFeatureVectors;
        Template::splitVectorsAndClasses(rawData, rawFeatureVectors, classes);

        int clusterCount = 5;
        QList<QVector<int> > classesInClusters;
        QList<QVector<Vector> > rawVectorsInClusters;
        BioDataProcessing::divideToNClusters(rawFeatureVectors, classes, clusterCount, rawVectorsInClusters, classesInClusters);

        QList<QVector<int> > trainClassesInClusters;
        QList<QVector<Vector> > trainVectorsInClusters;
        BioDataProcessing::divideToNClusters(rawVectorsInClusters[0], classesInClusters[0], 2,
                                             trainVectorsInClusters, trainClassesInClusters);

        PCA pca(trainVectorsInClusters[0]);
        pca.modesSelectionThreshold(0.995);
        ZScorePCAExtractor zscorePcaExtractor(pca, trainVectorsInClusters[1]);

        BatchEvaluationResult batchEvalResult = Evaluation::batch(rawVectorsInClusters, classesInClusters, zscorePcaExtractor, CorrelationMetric(), 1);
        foreach (const Evaluation &e, batchEvalResult.results)
        {
            qDebug() << e.eer;
        }
    }*/

    static void evaluateFilterBankFusion()
    {
        QString source = "depth";
        bool isGabor = false;
        QString filterType = (isGabor ? QString("gabor") : QString("gl"));

        // Declare variables
        QString path = "/home/stepo/data/frgc/spring2004/zbin-aligned2/" + source;
        double pcaThreshold = 0.995;
        int clusters = 5;

        // Create kernels
        QVector<Matrix> realWavelets;
        QVector<Matrix> imagWavelets;
        Face::Biometrics::FilterBankClassifier::addFilterKernels(realWavelets, imagWavelets, source, isGabor);

        Face::Biometrics::ScoreWeightedSumFusion fusion;

        QVector<QList<Face::Biometrics::Evaluation> > testData(clusters); // [cluster][method]
        // Load and process images
        {
            QVector<Matrix> images;
            QVector<int> classes;
            Face::LinAlg::Loader::loadMatrices(path, images, classes, "d", "*.gz");
            for (int i = 0; i < realWavelets.count(); i++)
            {
                qDebug() << source << i;
                QVector<Face::LinAlg::Vector> rawVectors;
                foreach(const Matrix &img, images)
                {
                    if (realWavelets[i].rows == 0)
                    {
                        rawVectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(
                                          Face::LinAlg::MatrixConverter::scale(img, 0.5));
                    }
                    else
                    {
                        rawVectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(
                                          Face::LinAlg::MatrixConverter::scale(
                                              Face::LinAlg::FilterBank::absResponse(
                                                  img, realWavelets[i], imagWavelets[i]), 0.5));
                    }
                }

                QList<QVector<Face::LinAlg::Vector> > vectorsInClusters;
                QList<QVector<int> > classesInClusters;
                Face::Biometrics::BioDataProcessing::divideToNClusters(rawVectors, classes, 5, vectorsInClusters, classesInClusters);

                Face::Biometrics::ZPCACorrW pcaCor(vectorsInClusters[0], pcaThreshold, vectorsInClusters[0]);
                pcaCor.extractor.serialize(filterType + "-" + source + "-" + QString::number(i) + "-pca",
                                           filterType + "-" + source + "-" + QString::number(i) + "-normParams");

                fusion.addComponent(Face::Biometrics::Evaluation(vectorsInClusters[1], classesInClusters[1], pcaCor.extractor, pcaCor.metric));

                for (int c = 1; c < clusters; c++)
                {
                    testData[c] << Face::Biometrics::Evaluation(vectorsInClusters[c], classesInClusters[c], pcaCor.extractor, pcaCor.metric);
                }
            }
        }

        // train fusion classifier
        qDebug() << "training fusion classifier";
        fusion.learn();
        fusion.serialize(filterType + "-" + source + "-wSumFusion");


        // evaluate
        for (int c = 1; c < clusters; c++)
        {
            Face::Biometrics::Evaluation result = fusion.evaluate(testData[c]);
            result.outputResults(filterType + "-" + source + "-" + QString::number(c-1), 50);
            qDebug() << result.eer;
        }
    }

    static void trainGaborFusion()
    {
        QString data = "depth";
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned2/" + data;
        qDebug() << srcDirPath;
        int kSize = 200;
        double pcaThreshold = 0.995;
        QVector<Matrix> srcImages;
        QVector<int> classes;
        Face::LinAlg::Loader::loadMatrices(srcDirPath, srcImages, classes, "d", "*.gz", 867);

        QList<QVector<Matrix> > srcImagesInClusters;
        QList<QVector<int> > classesInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters(srcImages, classes, 2, srcImagesInClusters, classesInClusters);

        for (int i = 0; i < srcImagesInClusters.count(); i++)
        {
            qDebug() << i << srcImagesInClusters[i].count();
        }

        QList<Face::Biometrics::Evaluation> components;
        QVector<Face::LinAlg::Vector> trainVectors;
        foreach(const Matrix &srcImg, srcImagesInClusters[0])
        {
            trainVectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(Face::LinAlg::MatrixConverter::scale(srcImg, 0.5));
        }
        QVector<Face::LinAlg::Vector> testVectors;
        foreach(const Matrix &srcImg, srcImagesInClusters[1])
        {
            testVectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(Face::LinAlg::MatrixConverter::scale(srcImg, 0.5));
        }

        Face::Biometrics::ZPCACorrW pcaCor(trainVectors, pcaThreshold, trainVectors);
        components << Face::Biometrics::Evaluation(testVectors, classesInClusters[1], pcaCor.extractor, pcaCor.metric);
        qDebug() << components.last().eer;

        Matrix realWavelet(kSize, kSize);
        Matrix imagWavelet(kSize, kSize);

        QMap<int, int> freqs; freqs[0] = 0;
        QMap<int, int> ornts; ornts[0] = 0;
        int index = 1;
        for (int freq = 4; freq <= 6; freq++)
        {
            for (int orientation = 1; orientation <= 8; orientation++)
            {
                freqs[index] = freq;
                ornts[index] = orientation;
                Face::LinAlg::Gabor::createWavelet(realWavelet, imagWavelet, freq, orientation);

                QVector<Face::LinAlg::Vector> trainVectors;
                foreach(const Matrix &srcImg, srcImagesInClusters[0])
                {
                    trainVectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(
                                        Face::LinAlg::MatrixConverter::scale(
                                            Face::LinAlg::Gabor::absResponse(
                                                srcImg, realWavelet, imagWavelet), 0.5));
                }

                Face::Biometrics::ZPCACorrW pcaCor(trainVectors, pcaThreshold, trainVectors);

                QVector<Face::LinAlg::Vector> vectors;
                foreach(const Matrix &srcImg, srcImagesInClusters[1])
                {
                    vectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(
                                   Face::LinAlg::MatrixConverter::scale(
                                       Face::LinAlg::Gabor::absResponse(
                                           srcImg, realWavelet, imagWavelet), 0.5));
                }

                components << Face::Biometrics::Evaluation(vectors, classesInClusters[1], pcaCor.extractor, pcaCor.metric);
                qDebug() << index << freq << orientation << components.last().eer;
                index++;
            }
        }

        Face::Biometrics::ScoreWeightedSumFusion fusion;
        QVector<int> keys = Face::Biometrics::ScoreLevelFusionWrapper::trainClassifier(fusion, components, true);
        QString fString, oString;
        foreach (int k, keys)
        {
            fString += " << " + QString::number(freqs[k]);
            oString += " << " + QString::number(ornts[k]);
        }
        qDebug() << fString;
        qDebug() << oString;
    }

    static void evaluateGaborFilterBanks()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned2/";
        QStringList sources; sources << "depth" << "eigencur" << "gauss" << "index" << "mean";
        //cv::Rect roi(25, 15, 100, 90);

        foreach (const QString &source, sources)
        {
            QVector<Matrix> srcImages;
            QVector<int> classes;
            Face::LinAlg::Loader::loadImages(srcDirPath + source, srcImages, &classes, "*.png", "d", 866); //, roi);

            for (int kSize = 200; kSize <= 200; kSize += 50)
            {
                Matrix realWavelet(kSize, kSize);
                Matrix imagWavelet(kSize, kSize);
                for (int freq = 3; freq <= 10; freq++)
                {
                    for (int orientation = 1; orientation <= 8; orientation++)
                    {
                        Face::LinAlg::Gabor::createWavelet(realWavelet, imagWavelet, freq, orientation);
                        QVector<Face::LinAlg::Vector> vectors;
                        foreach(const Matrix &srcImg, srcImages)
                        {
                            vectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(
                                           Face::LinAlg::Gabor::absResponse(srcImg, realWavelet, imagWavelet));
                        }

                        QList<QVector<int> > classesInClusters;
                        QList<QVector<Face::LinAlg::Vector> > vectorsInClusters;
                        Face::Biometrics::BioDataProcessing::divideToNClusters(vectors, classes, 2, vectorsInClusters, classesInClusters);
                        Face::LinAlg::PCA pca(vectorsInClusters[0]);
                        Face::Biometrics::ZScorePCAExtractor extractor(pca, vectorsInClusters[0]);
                        Face::Biometrics::Evaluation eval(vectorsInClusters[1], classesInClusters[1], extractor, Face::LinAlg::CorrelationMetric());
                        qDebug() << freq << orientation << eval.eer;
                    }
                    qDebug() << "";
                }
            }
        }
    }

    static void trainGaussLaguerreFusion()
    {
        QString data = "depth";
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned2/" + data;
        qDebug() << srcDirPath;
        double pcaThreshold = 0.995;
        QVector<Matrix> srcImages;
        QVector<int> classes;
        Face::LinAlg::Loader::loadMatrices(srcDirPath, srcImages, classes, "d", "*.gz", 867);

        QList<QVector<Matrix> > srcImagesInClusters;
        QList<QVector<int> > classesInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters(srcImages, classes, 2, srcImagesInClusters, classesInClusters);

        for (int i = 0; i < srcImagesInClusters.count(); i++)
        {
            qDebug() << i << srcImagesInClusters[i].count();
        }

        QList<Face::Biometrics::Evaluation> components;
        QVector<Face::LinAlg::Vector> trainVectors;
        foreach(const Matrix &srcImg, srcImagesInClusters[0])
        {
            trainVectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(Face::LinAlg::MatrixConverter::scale(srcImg, 0.5));
        }
        QVector<Face::LinAlg::Vector> testVectors;
        foreach(const Matrix &srcImg, srcImagesInClusters[1])
        {
            testVectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(Face::LinAlg::MatrixConverter::scale(srcImg, 0.5));
        }

        Face::Biometrics::ZPCACorrW pcaCor(trainVectors, pcaThreshold, trainVectors);
        components << Face::Biometrics::Evaluation(testVectors, classesInClusters[1], pcaCor.extractor, pcaCor.metric);
        qDebug() << components.last().eer;

        QMap<int, int> sizes; sizes[0] = 0;
        QMap<int, int> ns; ns[0] = 0;

        int k = 0;
        int index = 1;
        for (int kSize = 25; kSize <= 100; kSize += 25)
        {
            for (int n = 1; n <= 5; n++)
            {
                sizes[index] = kSize;
                ns[index] = n;

                Matrix realWavelet;
                Matrix imagWavelet;
                Face::LinAlg::GaussLaguerre::createWavelet(realWavelet, imagWavelet, kSize, n, k);

                QVector<Face::LinAlg::Vector> trainVectors;
                foreach(const Matrix &srcImg, srcImagesInClusters[0])
                {
                    trainVectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(
                                        Face::LinAlg::MatrixConverter::scale(
                                            Face::LinAlg::GaussLaguerre::absResponse(
                                                srcImg, realWavelet, imagWavelet), 0.5));
                }
                Face::Biometrics::ZPCACorrW pcaCor(trainVectors, pcaThreshold, trainVectors);

                QVector<Face::LinAlg::Vector> testVectors;
                foreach(const Matrix &srcImg, srcImagesInClusters[1])
                {
                    testVectors << Face::LinAlg::MatrixConverter::matrixToColumnVector(
                                       Face::LinAlg::MatrixConverter::scale(
                                           Face::LinAlg::GaussLaguerre::absResponse(
                                               srcImg, realWavelet, imagWavelet), 0.5));
                }
                components << Face::Biometrics::Evaluation(testVectors, classesInClusters[1], pcaCor.extractor, pcaCor.metric);

                qDebug() << index << kSize << n << components.last().eer;
                index++;
            }
        }

        Face::Biometrics::ScoreWeightedSumFusion fusion;
        QVector<int> keys = Face::Biometrics::ScoreLevelFusionWrapper::trainClassifier(fusion, components, true);
        QString sString, nString;
        foreach (int k, keys)
        {
            sString += " << " + QString::number(sizes[k]);
            nString += " << " + QString::number(ns[k]);
        }
        qDebug() << sString;
        qDebug() << nString;
    }

    static void evaluateFusionWrapper()
    {
        QString dir = "/home/stepo/git/face/test/frgc/filterBanks2/";
        QStringList units;
        units << "isocurves"
              << "gl-index" << "gl-mean" << "gl-gauss" << "gl-eigencur" << "gl-depth" << "gl-textureE"
              << "gabor-index" << "gabor-mean" << "gabor-gauss" << "gabor-eigencur" << "gabor-depth" << "gabor-textureE";

        //units << "gabor-textureE" << "gabor-depth" << "gabor-index" << "gabor-mean" << "gabor-gauss" << "gabor-eigencur"
        //      << "gl-textureE" << "gl-depth" << "gl-index" << "gl-mean" << "gl-gauss" << "gl-eigencur";

        Face::Biometrics::ScoreWeightedSumFusion fusion;
        QList<Face::Biometrics::Evaluation> trainComponents;
        foreach (const QString &unit, units)
        {
            QVector<double> trainGenScores = Face::LinAlg::Vector::fromFile(dir + unit + "-0-gen-scores").toQVector();
            QVector<double> trainImpScores = Face::LinAlg::Vector::fromFile(dir + unit + "-0-imp-scores").toQVector();
            trainComponents << Face::Biometrics::Evaluation(trainGenScores, trainImpScores);
            //qDebug() << Evaluation(trainGenScores, trainImpScores).eer;
        }

        QVector<int> selectedUnits = Face::Biometrics::ScoreLevelFusionWrapper::trainClassifier(fusion, trainComponents, true);

        for (int i = 0; i <= 3; i++)
        {
            QList<Face::Biometrics::Evaluation> testEvals;

            foreach (int unitIndex, selectedUnits)
            {
                QVector<double> testGenScores = Face::LinAlg::Vector::fromFile(dir + units[unitIndex] + "-" + QString::number(i) + "-gen-scores").toQVector();
                QVector<double> testImpScores = Face::LinAlg::Vector::fromFile(dir + units[unitIndex] + "-" + QString::number(i) + "-imp-scores").toQVector();
                testEvals << Face::Biometrics::Evaluation(testGenScores, testImpScores);
            }

            Face::Biometrics::Evaluation eval = fusion.evaluate(testEvals);
            qDebug() << eval.eer << eval.fnmrAtFmr(0.01) << eval.fnmrAtFmr(0.001) << eval.fnmrAtFmr(0.0001);

            eval.outputResultsDET(QString::number(i));
        }
    }

    static void evaluateFusionAll()
    {
        QString dir = "/home/stepo/git/face/test/frgc/filterBanks2/";
        QStringList units;
        units << "isocurves"
              << "gl-index" << "gl-mean" << "gl-gauss" << "gl-eigencur" << "gl-depth" << "gl-textureE"
              << "gabor-index" << "gabor-mean" << "gabor-gauss" << "gabor-eigencur" << "gabor-depth" << "gabor-textureE";

        {
            Face::Biometrics::ScoreSVMFusion fusion;
            foreach (const QString &unit, units)
            {
                QVector<double> trainGenScores = Face::LinAlg::Vector::fromFile(dir + unit + "-0-gen-scores").toQVector();
                QVector<double> trainImpScores = Face::LinAlg::Vector::fromFile(dir + unit + "-0-imp-scores").toQVector();
                fusion.addComponent(Face::Biometrics::Evaluation(trainGenScores, trainImpScores));
            }
            fusion.learn();
            fusion.serialize("final");
        }

        {
            Face::Biometrics::ScoreSVMFusion fusion2("final");
            for (int i = 0; i <= 3; i++)
            {
                QList<Face::Biometrics::Evaluation> testEvals;

                foreach (const QString &unit, units)
                {
                    QVector<double> testGenScores = Face::LinAlg::Vector::fromFile(dir + unit + "-" + QString::number(i) + "-gen-scores").toQVector();
                    QVector<double> testImpScores = Face::LinAlg::Vector::fromFile(dir + unit + "-" + QString::number(i) + "-imp-scores").toQVector();
                    testEvals << Face::Biometrics::Evaluation(testGenScores, testImpScores);
                }

                Face::Biometrics::Evaluation eval = fusion2.evaluate(testEvals);
                qDebug() << eval.eer << eval.fnmrAtFmr(0.01) << eval.fnmrAtFmr(0.001) << eval.fnmrAtFmr(0.0001) << eval.fnmrAtFmr(0.00001);

                eval.outputResultsDET(QString::number(i));
            }
        }
    }

    static void testFilterBankKernelSizes()
    {
        QString classifiersDir = "/home/stepo/git/face/test/frgc/classifiers/";
        Face::Biometrics::FaceClassifier faceClassifier(classifiersDir);

        foreach (QString bankName, faceClassifier.bankClassifiers.keys())
        {
            Face::Biometrics::FilterBanksClassifiers &bankClassifiers = faceClassifier.bankClassifiers[bankName];

            foreach (QString imgTypeName, bankClassifiers.dict.keys())
            {
                QVector<Matrix> &bank = bankClassifiers.dict[imgTypeName].realWavelets;

                int n = bank.count();
                for (int i = 0; i < n; i++)
                {
                    Matrix &kernel = bank[i];
                    if (kernel.rows == 0) continue;
                    double min,max;
                    cv::minMaxIdx(kernel, &min, &max);
                    QString name = bankName+"-"+imgTypeName+"-"+QString::number(i);
                    qDebug() << name;
                    cv::imshow(name.toStdString(), (kernel-min)/(max-min));
                    cv::waitKey(0);
                    cv::destroyWindow(name.toStdString());
                }
            }
        }
    }

    static void testSerializedClassifiers()
    {
        QString classifiersDir = "/home/stepo/git/face/test/frgc/classifiers/";
        Face::Biometrics::FaceClassifier faceClassifier(classifiersDir);

        QString dataDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned2/";
        QVector<QString> filenames = Face::LinAlg::Loader::listFiles(dataDirPath, "*.binz", Face::LinAlg::Loader::BaseFilename);
        QVector<int> classes;
        foreach (const QString &f, filenames)
        {
            classes << f.split("d")[0].toInt();
        }

        QList<QVector<QString> > filenamesInClusters;
        QList<QVector<int> > classesInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters(filenames, classes, 5, filenamesInClusters, classesInClusters);

        for (int cluster = 1; cluster <= filenamesInClusters.count(); cluster++)
        {
            QVector<Face::Biometrics::Face3DTemplate *> templates;
            for (int i = 0; i < filenamesInClusters[cluster].count(); i++)
            {
                int id = classesInClusters[cluster][i];
                QString fileName = filenamesInClusters[cluster][i] + ".binz";
                QString path = dataDirPath + fileName;
                Face::FaceData::Mesh faceMesh = Face::FaceData::Mesh::fromBINZ(path, false);
                templates << new Face::Biometrics::Face3DTemplate(id, faceMesh, faceClassifier);
                qDebug() << cluster << fileName << id << (i+1) << "/" << filenamesInClusters[cluster].count();

                //templates.last()->serialize(dataDirPath + "templates/" + filenamesInClusters[cluster][i], faceClassifier);
            }

            Face::Biometrics::Evaluation eval = faceClassifier.evaluate(templates);
            qDebug() << cluster << eval.eer;
        }
    }

    static void createTemplates()
    {
        Face::Biometrics::FaceClassifier classifier("../../test/frgc/classifiers");
        QString dir = "/home/stepo/data/frgc/spring2004/zbin-aligned2/";
        QVector<QString> files = Face::LinAlg::Loader::listFiles(dir, "*.binz", Face::LinAlg::Loader::BaseFilename);
        foreach (const QString &file, files)
        {
            Face::FaceData::Mesh face = Face::FaceData::Mesh::fromBINZ(dir + file + ".binz");
            Face::Biometrics::Face3DTemplate t(0, face, classifier);
            t.serialize(dir + "templates/" + file + ".xml.gz", classifier);
        }
    }

    static void evaluateSerializedTemplates()
    {
        Face::Biometrics::FaceClassifier classifier("../../test/frgc/classifiers");
        QString dir = "/home/stepo/data/frgc/spring2004/zbin-aligned2/templates";
        QVector<QString> files = Face::LinAlg::Loader::listFiles(dir, "*.xml.gz", Face::LinAlg::Loader::AbsoluteFull);
        QVector<Face::Biometrics::Face3DTemplate *> allTemplates;
        QVector<int> allClasses;
        qDebug() << "loading...";
        foreach (const QString &file, files)
        {
            int id = file.split('/').last().split('d').at(0).toInt();
            allTemplates << new Face::Biometrics::Face3DTemplate(id, file, classifier);
            allClasses << id;
        }

        qDebug() << "dividing...";
        int clustersCount = 5;
        QList<QVector<Face::Biometrics::Face3DTemplate *> > templatesInClusters;
        QList<QVector<int> > classesInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters(allTemplates, allClasses, clustersCount,
                                                               templatesInClusters, classesInClusters);

        qDebug() << "evaluating...";
        for (int i = 0; i < clustersCount; i++)
        {
            Face::Biometrics::Evaluation e = classifier.evaluate(templatesInClusters[i]);
            qDebug() << e.eer;
        }
    }
};

#endif
