#ifndef EVALUATE3DFRGC_H_
#define EVALUATE3DFRGC_H_

#include <qvector.h>
#include <qstring.h>
#include <qdebug.h>
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
        foreach (const QFileInfo &srcFileInfo, srcFiles)
        {
            Mesh mesh = Mesh::fromBIN(srcFileInfo.absoluteFilePath());
            aligner.icpAlign(mesh, 15);

            QString resultPath = outDirPath + srcFileInfo.baseName() + ".binz";
            mesh.writeBINZ(resultPath);

            Map texture = SurfaceProcessor::depthmap(mesh, converter, cv::Point2d(-100,-100), cv::Point2d(100,100), 1, Texture_I);
            QString resultTexturePath = outDirPath + srcFileInfo.baseName() + ".png";
            cv::imwrite(resultTexturePath.toStdString(), texture.toMatrix()*255);
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
        QVector<Template> rawData = IsoCurveProcessing::generateTemplates(data);
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
        //zscorePcaExtractor.serialize("../../test/isocurves/shifted-pca.yml", "../../test/isocurves/shifted-normparams.yml");

        QVector<Template> trainTemplates = Template::createTemplates(trainVectorsInClusters[1],
                                                                     trainClassesInClusters[1],
                                                                     zscorePcaExtractor);
        EERPotential eerPot(trainTemplates);

        for (double selThreshold = 0.0; selThreshold <= 0.3; selThreshold += 0.01)
        {
            CorrelationWeightedMetric corW;
            corW.w = eerPot.createSelectionWeightsBasedOnRelativeThreshold(selThreshold);
            Evaluation eCor(rawVectorsInClusters[1], classesInClusters[1], zscorePcaExtractor, corW);
            qDebug() << "selThreshold" << selThreshold << eCor.eer;
        }
    }

    static int createMaps()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        QDir srcDir(srcDirPath, "*.binz");
        QFileInfoList srcFiles = srcDir.entryInfoList();
        MapConverter converter;

        Matrix smoothKernel2 = KernelGenerator::gaussianKernel(7);
        //QVector<double> allZValues;
        //bool first = true;
        foreach (const QFileInfo &srcFileInfo, srcFiles)
        {
            if (QFile::exists(srcDirPath + "depth2/" + srcFileInfo.baseName() + ".png") &&
                QFile::exists(srcDirPath + "mean2/" + srcFileInfo.baseName() + ".png") &&
                QFile::exists(srcDirPath + "gauss2/" + srcFileInfo.baseName() + ".png") &&
                QFile::exists(srcDirPath + "index2/" + srcFileInfo.baseName() + ".png") &&
                QFile::exists(srcDirPath + "eigencur2/" + srcFileInfo.baseName() + ".png"))
            {
                continue;
            }

            Mesh mesh = Mesh::fromBINZ(srcFileInfo.absoluteFilePath());
            Map depthmap = SurfaceProcessor::depthmap(mesh, converter,
                                                      cv::Point2d(-75, -75),
                                                      cv::Point2d(75, 75),
                                                      2, ZCoord);
            //allZValues << depthmap.getUsedValues();

            depthmap.bandPass(-75, 0, false, false);
            Matrix depthImage = depthmap.toMatrix(0, -75, 0);
            QString out = srcDirPath + "depth2/" + srcFileInfo.baseName() + ".png";
            cv::imwrite(out.toStdString(), depthImage*255);

            Map smoothedDepthmap = depthmap;
            smoothedDepthmap.applyFilter(smoothKernel2, 7, true);
            CurvatureStruct cs = SurfaceProcessor::calculateCurvatures(smoothedDepthmap);

            cs.curvatureMean.bandPass(-0.1, 0.1, false, false);
            Matrix meanImage = cs.curvatureMean.toMatrix(0, -0.1, 0.1);
            out = srcDirPath + "mean2/" + srcFileInfo.baseName() + ".png";
            cv::imwrite(out.toStdString(), meanImage*255);

            cs.curvatureGauss.bandPass(-0.01, 0.01, false, false);
            Matrix gaussImage = cs.curvatureGauss.toMatrix(0, -0.01, 0.01);
            out = srcDirPath + "gauss2/" + srcFileInfo.baseName() + ".png";
            cv::imwrite(out.toStdString(), gaussImage*255);

            cs.curvatureIndex.bandPass(0, 1, false, false);
            Matrix indexImage = cs.curvatureIndex.toMatrix(0, 0, 1);
            out = srcDirPath + "index2/" + srcFileInfo.baseName() + ".png";
            cv::imwrite(out.toStdString(), indexImage*255);

            cs.curvaturePcl.bandPass(0, 0.0025, false, false);
            Matrix pclImage = cs.curvaturePcl.toMatrix(0, 0, 0.0025);
            out = srcDirPath + "eigencur2/" + srcFileInfo.baseName() + ".png";
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
        }

        QVector<Vector> allVectors;
        foreach(const Map &depth, allMaps)
        {
            allVectors << HistogramFeatures(depth, stripes, bins).toVector();
        }

        QList<QVector<int> > classesInClusters;
        QList<QVector<Vector> > vectorsInClusters;
        BioDataProcessing::divideToNClusters<Vector>(allVectors, allClasses, 2, vectorsInClusters, classesInClusters);

        ZScorePassExtractor zPassExtractor(vectorsInClusters[0]);

        Evaluation e(vectorsInClusters[1], classesInClusters[1], zPassExtractor, CorrelationMetric());
        qDebug() << "initial eer" << e.eer;

        QVector<Template> trainTemplates = Template::createTemplates(vectorsInClusters[0], classesInClusters[0], zPassExtractor);
        EERPotential eerPotential(trainTemplates);

        eerPotential.scores.toFile("eerPotentialTrain");

        CorrelationWeightedMetric corrW;
        double s = 0.8*(eerPotential.maxScore - eerPotential.minScore)+eerPotential.minScore;
        corrW.w = eerPotential.createSelectionWeights(s);
        Evaluation e2(vectorsInClusters[1], classesInClusters[1], zPassExtractor, corrW);
        qDebug() << "eerPotential scores" << s << "eer" << e2.eer;
    }

    static void evaluateImages()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        QStringList sources; sources << "depth2" << "eigencur2" << "gauss2"
                                     << "index2" << "mean2";
        QString source = "depth2";

        QVector<double> thresholds; thresholds << 0.90 << 0.91  << 0.92 << 0.93
                                               << 0.94 << 0.95  << 0.96 << 0.97
                                               << 0.98 << 0.985 << 0.99 << 0.995
                                               << 0.999 << 1;
        double threshold = 0.995;

        QVector<double> roiWidths; roiWidths << 50 << 50 << 60 << 60 << 60;
        QVector<double> roiUppers; roiUppers << 50 << 60 << 60 << 60 << 70;
        QVector<double> roiLowers; roiLowers << 30 << 30 << 30 << 40 << 40;
        cv::Rect roi(50, 30, 200, 180);
        /*cv::Rect roi(150-roiWidths[roiIndex]*2, 150-roiUppers[roiIndex]*2,
                     roiWidths[roiIndex]*4, roiUppers[roiIndex]*2+roiLowers[roiIndex]*2);*/

        QVector<Metrics*> metrics;
        metrics << (new CityblockMetric()) << (new EuclideanMetric())
                << (new CosineMetric()) << (new CorrelationMetric());

        QVector<WeightedMetric*> metricsW;
        metricsW << (new CityblockWeightedMetric()) << (new EuclideanWeightedMetric())
                 << (new CosineWeightedMetric()) << (new CorrelationWeightedMetric());

        QVector<Vector> vectors;
        QVector<Matrix> images;
        QVector<int> classes;
        Loader::loadImages(srcDirPath + source, images, &classes, "*.png", "d");
        int n = images.count();

        for (int i = 0; i < n; i++)
        {
            Matrix sub = images[i](roi);
            //cv::resize(images[i], images[i], cv::Size(images[i].cols/2, images[i].rows/2));

            vectors.append(MatrixConverter::matrixToColumnVector(sub));
        }

        QList<QVector<Vector> > vectorsInClusters;
        QList<QVector<int> > classesInClusters;
        BioDataProcessing::divideToNClusters(vectors, classes, 5, vectorsInClusters, classesInClusters);

        /*QVector<Template> templates = Template::createTemplates(vectorsInClusters[0],
                                                                classesInClusters[0],
                                                                PassExtractor());
        EERPotential eerMap(templates);
        Matrix eerPotentialImage = MatrixConverter::columnVectorToMatrix(eerMap.createWeights(), 200);
        cv::imshow("eerPotential", eerPotentialImage);
        cv::waitKey(0);
        exit(0);*/

        /*QList<QVector<Vector> > trainVectorsInClusters;
        QList<QVector<int> > trainClassesInClusters;
        BioDataProcessing::divideToNClusters(vectorsInClusters[0], classesInClusters[0], 2, trainVectorsInClusters, trainClassesInClusters);*/

        PCA pca(vectorsInClusters[0]);
        pca.modesSelectionThreshold(threshold);
        PCAExtractor pcaExtractor(pca);
        ZScorePCAExtractor zPcaExtractor(pca, vectorsInClusters[1]);

        qDebug() << "PCA" << Evaluation(vectorsInClusters[1],
                                        classesInClusters[1],
                                        pcaExtractor, CorrelationMetric()).eer;

        qDebug() << "z-PCA" << Evaluation(vectorsInClusters[1],
                                          classesInClusters[1],
                                          zPcaExtractor, CorrelationMetric()).eer;
    }

    static void evaluateFilterBanks()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/";
        QString source = "depth2";
        cv::Rect roi(50, 30, 200, 180);

        QVector<int> absIndicies; absIndicies << 18 << 19 << 20 << 21 << 22;
        QVector<int> realIndicies; realIndicies << 18 << 19 << 20 << 21 << 22;
        QVector<int> imagIndicies;

        QVector<Matrix> images;
        QVector<int> classes;
        Loader::loadImages(srcDirPath + source, images, &classes, "*.png", "d", 874);
        int n = images.count();
        //GaussLaguerre bank(13);
        Gabor bank(13);
        int filterCount = bank.realKernels.count();

        QVector<QVector<Vector> > responses(3*filterCount);
        for (int i = 0; i < n; i++)
        {
            Matrix sub = images[i](roi);
            cv::resize(sub, sub, cv::Size(sub.cols/2, sub.rows/2));

            QVector<Matrix> r = bank.getAbsRealImagResponse(sub);//, &absIndicies, &realIndicies, &imagIndicies);
            for (int j = 0; j < r.count(); j++)
            {
                if (i == 0)
                {
                    double min, max;
                    Common::getMinMax(r[j], min, max);
                    cv::imshow(QString::number(j).toStdString(), (r[j]-min)/(max-min));
                    cv::waitKey();
                }
                responses[j] << MatrixConverter::matrixToColumnVector(r[j]);
            }
            cv::destroyAllWindows();
        }

        for (int filterIndex = 0; filterIndex < filterCount*3; filterIndex++)
        {
            const QVector<Vector> vectors = responses[filterIndex];
            QList<QVector<Vector> > vectorsInClusters;
            QList<QVector<int> > classesInClusters;

            BioDataProcessing::divideToNClusters(vectors, classes, 2, vectorsInClusters, classesInClusters);
            PCA pca(vectorsInClusters[0]);
            ZScorePCAExtractor zPcaExtractor(pca, vectorsInClusters[0]);
            Evaluation eval(vectorsInClusters[1], classesInClusters[1], zPcaExtractor, CorrelationMetric());
            qDebug() << filterIndex << (filterIndex % filterCount) << eval.eer;
        }
    }

    static void evaluateFusion()
    {
        // ---------
        // Histogram
        // ---------
        QString histogramPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/histogram-20-20/";
        QVector<Vector> histogramVectors;
        QVector<int> histogramClasses;
        Loader::loadVectors(histogramPath, histogramVectors, histogramClasses, "d");

        QList<QVector<int> > histClassesInClusters;
        QList<QVector<Vector> > histVectorsInClusters;
        BioDataProcessing::divideToNClusters<Vector>(histogramVectors, histogramClasses, 5, histVectorsInClusters, histClassesInClusters);

        ZScorePassExtractor histZPassExtractor(histVectorsInClusters[0]);
        QVector<Template> histTrainTemplates = Template::createTemplates(histVectorsInClusters[0], histClassesInClusters[0], histZPassExtractor);
        EERPotential histEerPot(histTrainTemplates);

        CorrelationWeightedMetric histCorrW;
        histCorrW.w = histEerPot.createSelectionWeightsBasedOnRelativeThreshold(0.8);
        Evaluation histogramEval(histVectorsInClusters[1], histClassesInClusters[1], histZPassExtractor, histCorrW);
        histogramEval.outputResults("../../test/frgc/fusion/histogram", 50);
        qDebug() << "histogram EER" << histogramEval.eer;

        // ----------
        // Iso-curves
        // ----------
        QString curvesPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/isocurves2/";
        QVector<SubjectIsoCurves> curves = IsoCurveProcessing::readDirectory(curvesPath, "d", "*.xml");
        IsoCurveProcessing::selectIsoCurves(curves, 0, 5);
        QVector<Template> curveTemplates = IsoCurveProcessing::generateTemplates(curves);

        QVector<int> curveClasses;
        QVector<Vector> curveVectors;
        Template::splitVectorsAndClasses(curveTemplates, curveVectors, curveClasses);

        QList<QVector<int> > curveClassesInClusters;
        QList<QVector<Vector> > curveVectorsInClusters;
        BioDataProcessing::divideToNClusters(curveVectors, curveClasses, 5, curveVectorsInClusters, curveClassesInClusters);

        QList<QVector<int> > curveTrainClassesInClusters;
        QList<QVector<Vector> > curveTrainVectorsInClusters;
        BioDataProcessing::divideToNClusters(curveVectorsInClusters[0], curveClassesInClusters[0], 2,
                                             curveTrainVectorsInClusters, curveTrainClassesInClusters);

        PCA curvePca(curveTrainVectorsInClusters[0]);
        ZScorePCAExtractor curveZPcaExtractor(curvePca, curveTrainVectorsInClusters[1]);
        QVector<Template> curveTrainTemplates = Template::createTemplates(curveTrainVectorsInClusters[1],
                                                                          curveTrainClassesInClusters[1],
                                                                          curveZPcaExtractor);
        EERPotential curveEerPot(curveTrainTemplates);

        CorrelationWeightedMetric curvesCorrW;
        curvesCorrW.w = curveEerPot.createSelectionWeightsBasedOnRelativeThreshold(0.1);
        Evaluation curveEval(curveVectorsInClusters[1], curveClassesInClusters[1], curveZPcaExtractor, curvesCorrW);
        curveEval.outputResults("../../test/frgc/fusion/iso-curves", 50);
        qDebug() << "iso-curves" << curveEval.eer;

        // -----
        // Depth
        // -----
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

        QList<QVector<Vector> > depthVectorsInClusters;
        QList<QVector<int> > depthClassesInClusters;
        BioDataProcessing::divideToNClusters(depthVectors, depthClasses, 5, depthVectorsInClusters, depthClassesInClusters);

        PCA depthPca(depthVectorsInClusters[0]);
        depthPca.modesSelectionThreshold(0.995);
        ZScorePCAExtractor depthZPcaExtractor(depthPca, depthVectorsInClusters[0]);
        CorrelationMetric depthCorr;

        Evaluation depthEval(depthVectorsInClusters[1], depthClassesInClusters[1], depthZPcaExtractor, depthCorr);
        depthEval.outputResults("../../test/frgc/fusion/depth", 50);
        qDebug() << "depth" << depthEval.eer;

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
        svmFusion.addComponent(histVectorsInClusters[1], histClassesInClusters[1], histZPassExtractor, histCorrW);
        svmFusion.addComponent(curveVectorsInClusters[1], curveClassesInClusters[1], curveZPcaExtractor, curvesCorrW);
        svmFusion.addComponent(depthVectorsInClusters[1], depthClassesInClusters[1], depthZPcaExtractor, depthCorr);
        svmFusion.learn();

        QList<QVector<Vector> > vectorsInClusters;
        vectorsInClusters << histVectorsInClusters[2] << curveVectorsInClusters[2] << depthVectorsInClusters[2];
        Evaluation fusionEval = svmFusion.evaluate(vectorsInClusters, histClassesInClusters[2]);
        qDebug() << fusionEval.eer;
        fusionEval.outputResults("../../test/frgc/fusion/fusion", 50);
    }
};

#endif
