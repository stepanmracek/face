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
#include "facelib/mesh.h"
#include "facelib/surfaceprocessor.h"
#include "facelib/landmarkdetector.h"
#include "facelib/landmarks.h"
#include "facelib/facealigner.h"
#include "facelib/glwidget.h"
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
        QString outDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";

        Mesh mean = Mesh::fromOBJ("../../test/meanForAlign.obj");
        FaceAligner aligner(mean);
        MapConverter converter;

        QDir srcDir(srcDirPath, "*.bin");
        QFileInfoList srcFiles = srcDir.entryInfoList();
        //bool proceed = false;
        foreach (const QFileInfo &srcFileInfo, srcFiles)
        {
            //if (srcFileInfo.baseName().compare("04626d357") == 0) proceed = true;
            //if (!proceed) continue;

            Mesh mesh = Mesh::fromBIN(srcFileInfo.absoluteFilePath());
            aligner.icpAlign(mesh, 15);

            QString resultPath = outDirPath + srcFileInfo.baseName() + ".binz";
            mesh.writeBINZ(resultPath);

            Map texture = SurfaceProcessor::depthmap(mesh, converter, cv::Point2d(-100,-100), cv::Point2d(100,100), 1, Texture_I);
            QString resultTexturePath = outDirPath + srcFileInfo.baseName() + ".png";
            cv::imwrite(resultTexturePath.toStdString(), texture.toMatrix(0, 0, 255)*255);
        }
    }

    static int createIsoCurves()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        QString outDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/isocurves2/";
        QDir srcDir(srcDirPath, "*.binz");
        QFileInfoList srcFiles = srcDir.entryInfoList();
        foreach (const QFileInfo &srcFileInfo, srcFiles)
        {
            QString resultPath = outDirPath + srcFileInfo.baseName() + ".xml";
            if (QFile::exists(resultPath)) continue;

            Mesh mesh = Mesh::fromBINZ(srcFileInfo.absoluteFilePath());
            LandmarkDetector detector(mesh);
            Landmarks lm = detector.detect();
            cv::Point3d nosetip = lm.get(Landmarks::Nosetip);
            mesh.translate(-nosetip);

            MapConverter converter;
            Map depth = SurfaceProcessor::depthmap(mesh, converter, 2, ZCoord);
            Matrix gaussKernel = KernelGenerator::gaussianKernel(7);
            depth.applyFilter(gaussKernel, 3, true);

            QVector<VectorOfPoints> isoCurves;
            int startD = 10;
            for (int d = startD; d <= 100; d += 10)
            {
                VectorOfPoints isoCurve = SurfaceProcessor::isoGeodeticCurve(depth, converter, cv::Point3d(0,20,0), d, 100, 2);
                isoCurves << isoCurve;
            }

            Serialization::serializeVectorOfPointclouds(isoCurves, resultPath);
        }
    }

    static void evaluateIsoCurves()
    {
        int modulo = 1;
        double pcaThreshold = 1.0;
        QVector<double> thresholds; thresholds << 0.90 << 0.91  << 0.92 << 0.93
                                               << 0.94 << 0.95  << 0.96 << 0.97
                                               << 0.98 << 0.985 << 0.99 << 0.995
                                               << 0.999 << 1;

        QString dirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/isocurves2/";
        QVector<SubjectIsoCurves> data = IsoCurveProcessing::readDirectory(dirPath, "d", "*.xml");
        IsoCurveProcessing::selectIsoCurves(data, 0, 5);
        IsoCurveProcessing::stats(data);
        IsoCurveProcessing::sampleIsoCurvePoints(data, modulo);
        QVector<Template> rawData = IsoCurveProcessing::generateTemplates(data, false);
        //QVector<Template> rawData = IsoCurveProcessing::generateEuclDistanceTemplates(data);

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
        pca.modesSelectionThreshold(pcaThreshold);
        //PCAExtractor pcaExtractor(pca);
        ZScorePCAExtractor zscorePcaExtractor(pca, trainVectorsInClusters[1]);
        zscorePcaExtractor.serialize("../../test/isocurves/shifted-pca.yml", "../../test/isocurves/shifted-normparams.yml");

        QVector<Template> trainTemplates = Template::createTemplates(trainVectorsInClusters[1],
                                                                     trainClassesInClusters[1],
                                                                     zscorePcaExtractor);
        EERPotential eerPot(trainTemplates);

        CorrelationWeightedMetric corW;
        double bestThreshold;
        double bestEER = 1;
        for (double selThreshold = 0.0; selThreshold <= 0.3; selThreshold += 0.01)
        {
            corW.w = eerPot.createSelectionWeightsBasedOnRelativeThreshold(selThreshold);
            Evaluation eCor(rawVectorsInClusters[1], classesInClusters[1], zscorePcaExtractor, corW);

            if (eCor.eer < bestEER)
            {
                bestEER = eCor.eer;
                bestThreshold = selThreshold;
            }

            qDebug() << "selThreshold" << selThreshold << eCor.eer;
        }

        corW.w = eerPot.createSelectionWeightsBasedOnRelativeThreshold(bestThreshold);
        BatchEvaluationResult batchResult = Evaluation::batch(rawVectorsInClusters, classesInClusters,
                                                              zscorePcaExtractor, corW, 2);
        qDebug() << batchResult.meanEER << batchResult.stdDevOfEER;
        foreach(const Evaluation &e, batchResult.results)
        {
            qDebug() << e.eer;
        }
    }

    static int createMaps()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        QDir srcDir(srcDirPath, "*.binz");
        QFileInfoList srcFiles = srcDir.entryInfoList();
        MapConverter converter;

        Matrix smoothKernel2 = KernelGenerator::gaussianKernel(5);
        //QVector<double> allZValues;
        //bool first = true;
        foreach (const QFileInfo &srcFileInfo, srcFiles)
        {
            /*if (QFile::exists(srcDirPath + "mean/" + srcFileInfo.baseName() + ".png") &&
                QFile::exists(srcDirPath + "gauss/" + srcFileInfo.baseName() + ".png") &&
                QFile::exists(srcDirPath + "index/" + srcFileInfo.baseName() + ".png") &&
                QFile::exists(srcDirPath + "eigencur/" + srcFileInfo.baseName() + ".png"))
            {
                continue;
            }*/

            Mesh mesh = Mesh::fromBINZ(srcFileInfo.absoluteFilePath());
            Map depthmap = SurfaceProcessor::depthmap(mesh, converter,
                                                      cv::Point2d(-75, -75),
                                                      cv::Point2d(75, 75),
                                                      1, ZCoord);
            //allZValues << depthmap.getUsedValues();

            QString out;
            depthmap.bandPass(-75, 0, false, false);
            Matrix depthImage = depthmap.toMatrix(0, -75, 0);
            out = srcDirPath + "depth/" + srcFileInfo.baseName() + ".png";
            cv::imwrite(out.toStdString(), depthImage*255);

            Map smoothedDepthmap = depthmap;
            smoothedDepthmap.applyFilter(smoothKernel2, 7, true);
            CurvatureStruct cs = SurfaceProcessor::calculateCurvatures(smoothedDepthmap);

            cs.curvatureMean.bandPass(-0.1, 0.1, false, false);
            Matrix meanImage = cs.curvatureMean.toMatrix(0, -0.1, 0.1);
            out = srcDirPath + "mean/" + srcFileInfo.baseName() + ".png";
            cv::imwrite(out.toStdString(), meanImage*255);

            cs.curvatureGauss.bandPass(-0.01, 0.01, false, false);
            Matrix gaussImage = cs.curvatureGauss.toMatrix(0, -0.01, 0.01);
            out = srcDirPath + "gauss/" + srcFileInfo.baseName() + ".png";
            cv::imwrite(out.toStdString(), gaussImage*255);

            cs.curvatureIndex.bandPass(0, 1, false, false);
            Matrix indexImage = cs.curvatureIndex.toMatrix(0, 0, 1);
            out = srcDirPath + "index/" + srcFileInfo.baseName() + ".png";
            cv::imwrite(out.toStdString(), indexImage*255);

            cs.curvaturePcl.bandPass(0, 0.0025, false, false);
            Matrix pclImage = cs.curvaturePcl.toMatrix(0, 0, 0.0025);
            out = srcDirPath + "eigencur/" + srcFileInfo.baseName() + ".png";
            cv::imwrite(out.toStdString(), pclImage*255);
        }

        /*Histogram zValuesHistogram(allZValues, 20, false);
        qDebug() << "mean" << zValuesHistogram.mean;
        qDebug() << "stdDev" << zValuesHistogram.stdDev;
        qDebug() << "min" << zValuesHistogram.minValue;
        qDebug() << "max" << zValuesHistogram.maxValue;
        Common::savePlot(zValuesHistogram.histogramValues, zValuesHistogram.histogramCounter, "zValuesHistogram");*/
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
            ImageGrayscale full = cv::imread(fileInfo.absoluteFilePath().toStdString(), cv::IMREAD_GRAYSCALE);
            cv::GaussianBlur(full, full, cv::Size(21,21), 0);
            ImageGrayscale cropped = full(cv::Rect(40, 20, 220, 180));
            allImages << cropped;
            allClasses << fileInfo.baseName().split('d')[0].toInt();
        }

        QList<QVector<ImageGrayscale> > imagesInClusters;
        QList<QVector<int> > classesInClusters;
        BioDataProcessing::divideToNClusters<ImageGrayscale>(allImages, allClasses, 10, imagesInClusters, classesInClusters);

        for (int i = 0; i < 3; i++)
        {
            qDebug() << "Evaluating" << i << imagesInClusters[i].count();
            for (int stripes = 3; stripes <= 50; stripes++)
            {
                for (int bins = 3; bins <= 50; bins++)
                {
                    QVector<Vector> rawVectors;
                    foreach(const ImageGrayscale &image, imagesInClusters[i])
                    {
                        HistogramFeatures features(image, stripes, bins);
                        rawVectors << features.toVector();
                    }

                    Evaluation e(rawVectors, classesInClusters[i], PassExtractor(), CityblockMetric());
                    qDebug() << stripes << bins << e.eer; //  results.meanEER << "+-" << results.stdDevOfEER;
                }
                qDebug() << "";
            }
        }
    }

    static void evaluateHistogramFeatures()
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

        /*QVector<int> startx; startx << -50 << -50 << -60 << -60 << -60;
        QVector<int> starty; starty << -30 << -30 << -30 << -40 << -40;
        QVector<int> endx;   endx   <<  50 <<  50 <<  60 <<  60 <<  60;
        QVector<int> endy;   endy   <<  50 <<  60 <<  60 <<  60 <<  70;

        QVector<int> kSizes; kSizes << 3 << 5 << 7 << 9;
        QVector<int> filterRepeats; filterRepeats << 1 << 2 << 3;

        QVector<Metrics*> metrics;
        metrics << new CityblockMetric() << new SumOfSquareDifferences() << new CosineMetric() << new CorrelationMetric();*/

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

        /*CorrelationWeightedMetric corrW;
        double s = 0.8;
        Vector selectionWeights = eerPotential.createSelectionWeightsBasedOnRelativeThreshold(s);
        selectionWeights.toFile("../../test/frgc/histogram/selectionWeights");
        corrW.w = selectionWeights;

        Evaluation e2(vectorsInClusters[1], classesInClusters[1], zPassExtractor, corrW);
        qDebug() << "eerPotential scores" << s << "eer" << e2.eer;*/
    }

    static void evaluateTextures()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        double threshold = 0.995;
        cv::Rect roi(50, 40, 100, 90);
        CorrelationMetric metric;

        QVector<Vector> vectors;
        QVector<int> classes;
        Loader::loadImages(srcDirPath, vectors, &classes, "*.png", "d", -1, roi);

        QList<QVector<Vector> > vectorsInClusters;
        QList<QVector<int> > classesInClusters;
        BioDataProcessing::divideToNClusters(vectors, classes, 5, vectorsInClusters, classesInClusters);

        PCA pca(vectorsInClusters[0]);
        pca.modesSelectionThreshold(threshold);
        ZScorePCAExtractor extractor(pca, vectorsInClusters[1]);
        extractor.serialize("../../test/frgc/texture/pca.yml", "../../test/frgc/texture/normparams.yml");

        //qDebug() << Evaluation(vectorsInClusters[1], classesInClusters[1], extractor, metric).eer;
        BatchEvaluationResult batchEval = Evaluation::batch(vectorsInClusters, classesInClusters, extractor, metric, 2);
        foreach(const Evaluation &eval, batchEval.results)
        {
            qDebug() << eval.eer;
        }

        //eval.outputResultsGenuineScores("texture-genuine-scores");
        //eval.outputResultsImpostorScores("texture-impostor-scores");
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

        QList<ScoreLevelFusionComponent> components;
        QList<QVector<Vector> > allData;
        QVector<int> testClasses;
        foreach (const QString &source, sources)
        {
            QVector<Vector> vectors;
            QVector<int> classes;
            Loader::loadImages(srcDirPath + source, vectors, &classes, "*.png", "d", -1, roi);

            QList<QVector<Vector> > vectorsInClusters;
            QList<QVector<int> > classesInClusters;
            BioDataProcessing::divideToNClusters(vectors, classes, 5, vectorsInClusters, classesInClusters);

            PCA pca(vectorsInClusters[0]);
            pca.modesSelectionThreshold(threshold);
            ZScorePCAExtractor *zPcaExtractor = new ZScorePCAExtractor(pca, vectorsInClusters[0]);
            zPcaExtractor->serialize("../../test/frgc/" + source + "/pca.yml",
                                     "../../test/frgc/" + source + "/normparams.yml");

            Metrics *metrics = new CorrelationMetric();

            Evaluation eval(vectorsInClusters[1],
                            classesInClusters[1],
                            *zPcaExtractor, *metrics);
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
        QVector<Vector> vectors;
        Loader::loadImages(srcDirPath + source, images, &classes, "*.png", "d", 874);
        int n = images.count();

        for (int i = 0; i < n; i++)
        {
            Matrix sub = images[i](roi);
            cv::resize(sub, sub, cv::Size(sub.cols/4, sub.rows/4));
            vectors << MatrixConverter::matrixToColumnVector(sub);
        }

        QList<QVector<Vector> > vectorsInClusters;
        QList<QVector<int> > classesInClusters;
        BioDataProcessing::divideToNClusters(vectors, classes, 2, vectorsInClusters, classesInClusters);

        ZScorePassExtractor extractor(vectorsInClusters[0]);
        QVector<Template> trainTemplates = Template::joinVectorsAndClasses(vectorsInClusters[0], classesInClusters[0]);
        EERPotential potential(trainTemplates);
        CorrelationWeightedMetric metric;
        for (double t = 0.05; t <= 0.95; t += 0.05) // 0.3 je nejlepsi
        {
            metric.w = potential.createSelectionWeightsBasedOnRelativeThreshold(t);
            qDebug() << t << Evaluation(vectorsInClusters[1], classesInClusters[1], extractor, metric).eer;
        }
    }

    static int createCurves()
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

            /*int argc;
            char **argv;
            QApplication app(argc, argv);
            GLWidget w;
            w.addFace(&mesh);
            foreach (VectorOfPoints v, curves)
            {
                w.addCurve(v);
            }
            w.show();
            return app.exec();*/

            //maps << depth;
            //if (maps.count() == 847) break;
        }

        /*Map mean(120, 90);
        mean.setAll(0);
        foreach(const Map &m, maps)
        {
            mean.add(m);
            //cv::imshow("mean", mean.toMatrix());
            //cv::waitKey();
        }
        mean.linearTransform(1.0/maps.count(), 0);
        int count = 0;
        for (int i = 0; i < mean.w*mean.h; i++)
        {
            if (mean.flags[i]) count++;
        }
        qDebug() << count;
        cv::imshow("mean", mean.toMatrix());
        cv::waitKey();*/
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
    }

    static void evaluateGaborFusion()
    {
        // Declare variables
        QString indexPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/index";
        QString depthPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/depth";
        cv::Rect roi(25, 15, 100, 90);
        int kSize = 200;
        int clusters = 5;

        // Create index kernels
        QVector<Matrix> indexRealWavelets;
        QVector<Matrix> indexImagWavelets;
        for (int i = 0; i < 7; i++)
        {
            indexRealWavelets << Matrix(kSize, kSize);
            indexImagWavelets << Matrix(kSize, kSize);
        }
        Gabor::createWavelet(indexRealWavelets[0], indexImagWavelets[0], 4, 8);
        Gabor::createWavelet(indexRealWavelets[1], indexImagWavelets[1], 5, 2);
        Gabor::createWavelet(indexRealWavelets[2], indexImagWavelets[2], 5, 4);
        Gabor::createWavelet(indexRealWavelets[3], indexImagWavelets[3], 5, 7);
        Gabor::createWavelet(indexRealWavelets[4], indexImagWavelets[4], 5, 8);
        Gabor::createWavelet(indexRealWavelets[5], indexImagWavelets[5], 6, 3);
        Gabor::createWavelet(indexRealWavelets[6], indexImagWavelets[6], 6, 5);

        // Create depth kernels
        QVector<Matrix> depthRealWavelets;
        QVector<Matrix> depthImagWavelets;
        for (int i = 0; i < 6; i++)
        {
            depthRealWavelets << Matrix(kSize, kSize);
            depthImagWavelets << Matrix(kSize, kSize);
        }
        Gabor::createWavelet(depthRealWavelets[0], depthImagWavelets[0], 5, 1);
        Gabor::createWavelet(depthRealWavelets[1], depthImagWavelets[1], 5, 2);
        Gabor::createWavelet(depthRealWavelets[2], depthImagWavelets[2], 5, 4);
        Gabor::createWavelet(depthRealWavelets[3], depthImagWavelets[3], 6, 3);
        Gabor::createWavelet(depthRealWavelets[4], depthImagWavelets[4], 6, 6);
        Gabor::createWavelet(depthRealWavelets[5], depthImagWavelets[5], 6, 8);

        CorrelationMetric metric;
        ScoreSVMFusion fusion;

        QVector<QList<Templates> > testTemplates(clusters - 1); // [cluster][method][subject]
        // Load and process index images
        {
            QVector<Matrix> indexImages;
            QVector<int> classes;
            Loader::loadImages(indexPath, indexImages, &classes, "*.png", "d", -1, roi);
            for (int i = 0; i < indexRealWavelets.count(); i++)
            {
                qDebug() << "index" << i;
                QVector<Vector> rawVectors;
                foreach(const Matrix &img, indexImages)
                {
                    rawVectors << MatrixConverter::matrixToColumnVector(Gabor::absResponse(img, indexRealWavelets[0], indexImagWavelets[0]));
                }

                QList<QVector<Vector> > vectorsInClusters;
                QList<QVector<int> > classesInClusters;
                BioDataProcessing::divideToNClusters(rawVectors, classes, 5, vectorsInClusters, classesInClusters);
                PCA pca(vectorsInClusters[0]);
                ZScorePCAExtractor extractor(pca, vectorsInClusters[0]);
                fusion.addComponent(ScoreLevelFusionComponent(
                                        Template::createTemplates(
                                            vectorsInClusters[1], classesInClusters[1], extractor), &metric));

                for (int c = 0; c < clusters-1; c++)
                {
                    testTemplates[c] << Template::createTemplates(vectorsInClusters[c+1], classesInClusters[c+1], extractor);
                }
            }
        }

        // Load and process depth images
        {
            QVector<Matrix> depthImages;
            QVector<int> classes;
            Loader::loadImages(depthPath, depthImages, &classes, "*.png", "d", -1, roi);
            for (int i = 0; i < depthRealWavelets.count(); i++)
            {
                qDebug() << "depth" << i;
                QVector<Vector> rawVectors;
                foreach(const Matrix &img, depthImages)
                {
                    rawVectors << MatrixConverter::matrixToColumnVector(Gabor::absResponse(img, depthRealWavelets[0], depthImagWavelets[0]));
                }

                QList<QVector<Vector> > vectorsInClusters;
                QList<QVector<int> > classesInClusters;
                BioDataProcessing::divideToNClusters(rawVectors, classes, clusters, vectorsInClusters, classesInClusters);
                PCA pca(vectorsInClusters[0]);
                ZScorePCAExtractor extractor(pca, vectorsInClusters[0]);
                fusion.addComponent(ScoreLevelFusionComponent(
                                        Template::createTemplates(
                                            vectorsInClusters[1], classesInClusters[1], extractor), &metric));

                for (int c = 0; c < clusters-1; c++)
                {
                    testTemplates[c] << Template::createTemplates(vectorsInClusters[c+1], classesInClusters[c+1], extractor);
                }
            }

            // plain images
            qDebug() << "depth plain";
            QVector<Vector> rawVectors;
            foreach(const Matrix &img, depthImages)
            {
                rawVectors << MatrixConverter::matrixToColumnVector(img);
            }

            QList<QVector<Vector> > vectorsInClusters;
            QList<QVector<int> > classesInClusters;
            BioDataProcessing::divideToNClusters(rawVectors, classes, clusters, vectorsInClusters, classesInClusters);
            PCA pca(vectorsInClusters[0]);
            ZScorePCAExtractor extractor(pca, vectorsInClusters[0]);
            fusion.addComponent(ScoreLevelFusionComponent(
                                    Template::createTemplates(
                                        vectorsInClusters[1], classesInClusters[1], extractor), &metric));

            for (int c = 0; c < clusters - 1; c++)
            {
                testTemplates[c] << Template::createTemplates(vectorsInClusters[c+1], classesInClusters[c+1], extractor);
            }
        }

        // train fusion classifier
        qDebug() << "training fusion classifier";
        fusion.learn();

        // evaluate
        for (int c = 0; c < clusters - 1; c++)
        {
            qDebug() << fusion.evaluate(testTemplates[c]).eer;
        }
    }

    static void trainGaborFusion()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/mean";
        cv::Rect roi(25, 15, 100, 90);
        int kSize = 200;
        QVector<Matrix> srcImages;
        QVector<int> classes;
        Loader::loadImages(srcDirPath, srcImages, &classes, "*.png", "d", 866, roi);

        QList<QVector<Matrix> > srcImagesInClusters;
        QList<QVector<int> > classesInClusters;
        BioDataProcessing::divideToNClusters(srcImages, classes, 2, srcImagesInClusters, classesInClusters);

        QList<ScoreLevelFusionComponent> components;
        QVector<Vector> trainVectors;
        foreach(const Matrix &srcImg, srcImagesInClusters[0])
            trainVectors << MatrixConverter::matrixToColumnVector(srcImg);
        QVector<Vector> testVectors;
        foreach(const Matrix &srcImg, srcImagesInClusters[1])
            testVectors << MatrixConverter::matrixToColumnVector(srcImg);

        PCA pca(trainVectors);
        pca.modesSelectionThreshold();
        ZScorePCAExtractor extractor(pca, trainVectors);
        Templates testTempates = Template::createTemplates(testVectors, classesInClusters[1], extractor);
        components << ScoreLevelFusionComponent(testTempates, new CorrelationMetric());

        Matrix realWavelet(kSize, kSize);
        Matrix imagWavelet(kSize, kSize);

        for (int freq = 4; freq <= 6; freq++)
        {
            for (int orientation = 1; orientation <= 8; orientation++)
            {
                qDebug() << freq << orientation;
                Gabor::createWavelet(realWavelet, imagWavelet, freq, orientation);

                QVector<Vector> trainVectors;
                foreach(const Matrix &srcImg, srcImagesInClusters[0])
                {
                    trainVectors << MatrixConverter::matrixToColumnVector(Gabor::absResponse(srcImg, realWavelet, imagWavelet));
                }
                PCA pca(trainVectors);
                ZScorePCAExtractor extractor(pca, trainVectors);

                QVector<Vector> vectors;
                foreach(const Matrix &srcImg, srcImagesInClusters[1])
                {
                    vectors << MatrixConverter::matrixToColumnVector(Gabor::absResponse(srcImg, realWavelet, imagWavelet));
                }
                components << ScoreLevelFusionComponent(Template::createTemplates(vectors, classesInClusters[1], extractor), new CorrelationMetric());
            }
        }

        ScoreSVMFusion fusion;
        ScoreLevelFusionWrapper::trainClassifier(fusion, components);
    }

    static void evaluateGaborFilterBanks()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        QStringList sources; sources << "depth" << "eigencur" << "gauss" << "index" << "mean";
        cv::Rect roi(25, 15, 100, 90);

        foreach (const QString &source, sources)
        {
            QVector<Matrix> srcImages;
            QVector<int> classes;
            Loader::loadImages(srcDirPath + source, srcImages, &classes, "*.png", "d", 866, roi);

            for (int kSize = 200; kSize <= 200; kSize += 50)
            {
                Matrix realWavelet(kSize, kSize);
                Matrix imagWavelet(kSize, kSize);
                for (int freq = 3; freq <= 10; freq++)
                {
                    for (int orientation = 1; orientation <= 8; orientation++)
                    {
                        Gabor::createWavelet(realWavelet, imagWavelet, freq, orientation);
                        QVector<Vector> vectors;
                        foreach(const Matrix &srcImg, srcImages)
                        {
                            vectors << MatrixConverter::matrixToColumnVector(Gabor::absResponse(srcImg, realWavelet, imagWavelet));
                        }

                        QList<QVector<int> > classesInClusters;
                        QList<QVector<Vector> > vectorsInClusters;
                        BioDataProcessing::divideToNClusters(vectors, classes, 2, vectorsInClusters, classesInClusters);
                        PCA pca(vectorsInClusters[0]);
                        ZScorePCAExtractor extractor(pca, vectorsInClusters[0]);
                        Evaluation eval(vectorsInClusters[1], classesInClusters[1], extractor, CorrelationMetric());
                        qDebug() << freq << orientation << eval.eer;
                    }
                    qDebug() << "";
                }
            }
        }
    }

    static void evaluateFusion()
    {
        /*int clusters = 5;
        // ---------
        // Histogram
        // ---------
        QList<QVector<int> > histClassesInClusters;
        QList<QVector<Vector> > histVectorsInClusters;
        ZScorePassExtractor histZPassExtractor;
        CorrelationWeightedMetric histCorrW;
        {
            QString histogramPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/histogram-20-20/";
            QVector<Vector> histogramVectors;
            QVector<int> histogramClasses;
            Loader::loadVectors(histogramPath, histogramVectors, histogramClasses, "d");

            BioDataProcessing::divideToNClusters<Vector>(histogramVectors, histogramClasses, clusters, histVectorsInClusters, histClassesInClusters);

            histZPassExtractor = ZScorePassExtractor(histVectorsInClusters[0]);
            QVector<Template> histTrainTemplates = Template::createTemplates(histVectorsInClusters[0], histClassesInClusters[0], histZPassExtractor);
            EERPotential histEerPot(histTrainTemplates);

            histCorrW.w = histEerPot.createSelectionWeightsBasedOnRelativeThreshold(0.8);
            Evaluation histogramEval(histVectorsInClusters[1], histClassesInClusters[1], histZPassExtractor, histCorrW);
            //histogramEval.outputResults("../../test/frgc/fusion/histogram", 50);
            qDebug() << "histogram EER" << histogramEval.eer;
        }

        // ----------
        // Iso-curves
        // ----------
        QList<QVector<int> > curveClassesInClusters;
        QList<QVector<Vector> > curveVectorsInClusters;
        ZScorePCAExtractor curveZPcaExtractor;
        CorrelationWeightedMetric curvesCorrW;
        {
            QString curvesPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/isocurves2/";
            QVector<SubjectIsoCurves> curves = IsoCurveProcessing::readDirectory(curvesPath, "d", "*.xml");
            IsoCurveProcessing::selectIsoCurves(curves, 0, 5);
            QVector<Template> curveTemplates = IsoCurveProcessing::generateTemplates(curves, false);

            QVector<int> curveClasses;
            QVector<Vector> curveVectors;
            Template::splitVectorsAndClasses(curveTemplates, curveVectors, curveClasses);

            BioDataProcessing::divideToNClusters(curveVectors, curveClasses, clusters, curveVectorsInClusters, curveClassesInClusters);

            QList<QVector<int> > curveTrainClassesInClusters;
            QList<QVector<Vector> > curveTrainVectorsInClusters;
            BioDataProcessing::divideToNClusters(curveVectorsInClusters[0], curveClassesInClusters[0], 2,
                                                 curveTrainVectorsInClusters, curveTrainClassesInClusters);

            PCA curvePca(curveTrainVectorsInClusters[0]);
            curveZPcaExtractor = ZScorePCAExtractor(curvePca, curveTrainVectorsInClusters[1]);
            QVector<Template> curveTrainTemplates = Template::createTemplates(curveTrainVectorsInClusters[1],
                                                                              curveTrainClassesInClusters[1],
                                                                              curveZPcaExtractor);
            EERPotential curveEerPot(curveTrainTemplates);

            curvesCorrW.w = curveEerPot.createSelectionWeightsBasedOnRelativeThreshold(0.1);
            Evaluation curveEval(curveVectorsInClusters[1], curveClassesInClusters[1], curveZPcaExtractor, curvesCorrW);
            //curveEval.outputResults("../../test/frgc/fusion/iso-curves", 50);
            qDebug() << "iso-curves" << curveEval.eer;
        }

        // -----
        // Depth
        // -----
        QList<QVector<Vector> > depthVectorsInClusters;
        QList<QVector<int> > depthClassesInClusters;
        ZScorePCAExtractor depthZPcaExtractor;
        CorrelationMetric depthCorr;
        {
            QString depthPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/depth2";
            cv::Rect depthRoi(50, 30, 200, 180);

            QVector<Vector> depthVectors;
            QVector<Matrix> depthImages;
            QVector<int> depthClasses;
            Loader::loadImages(depthPath, depthImages, &depthClasses, "*.png", "d");
            for (int i = 0; i < depthImages.count(); i++)
            {
                depthVectors.append(MatrixConverter::matrixToColumnVector(depthImages[i](depthRoi)));
            }

            BioDataProcessing::divideToNClusters(depthVectors, depthClasses, clusters, depthVectorsInClusters, depthClassesInClusters);

            PCA depthPca(depthVectorsInClusters[0]);
            depthPca.modesSelectionThreshold(0.995);
            depthZPcaExtractor = ZScorePCAExtractor(depthPca, depthVectorsInClusters[0]);

            Evaluation depthEval(depthVectorsInClusters[1], depthClassesInClusters[1], depthZPcaExtractor, depthCorr);
            //depthEval.outputResults("../../test/frgc/fusion/depth", 50);
            qDebug() << "depth" << depthEval.eer;
        }

        // ------
        // Fusion
        // ------

        int n = histClassesInClusters[1].count();
        assert(n == curveClassesInClusters[1].count());
        assert(n == depthClassesInClusters[1].count());
        for (int i = 0; i < n; i++)
        {
            assert(histClassesInClusters[1][i] = curveClassesInClusters[1][i]);
            assert(histClassesInClusters[1][i] = depthClassesInClusters[1][i]);
        }

        ScoreWeightedSumFusion svmFusion;
        svmFusion.addComponent(ScoreLevelFusionComponent(histVectorsInClusters[1],
                               histClassesInClusters[1], &histZPassExtractor, &histCorrW));
        svmFusion.addComponent(ScoreLevelFusionComponent(curveVectorsInClusters[1],
                               curveClassesInClusters[1], &curveZPcaExtractor, &curvesCorrW));
        svmFusion.addComponent(ScoreLevelFusionComponent(depthVectorsInClusters[1],
                               depthClassesInClusters[1], &depthZPcaExtractor, &depthCorr));
        svmFusion.learn();

        for (int i = 2; i < clusters; i++)
        {
            QList<QVector<Vector> > vectors;
            vectors << histVectorsInClusters[i] << curveVectorsInClusters[i] << depthVectorsInClusters[i];
            Evaluation fusionEval = svmFusion.evaluate(vectors, histClassesInClusters[i]);
            qDebug() << fusionEval.eer;
        }
        //fusionEval.outputResults("../../test/frgc/fusion/fusion", 50);*/
    }
};

#endif
