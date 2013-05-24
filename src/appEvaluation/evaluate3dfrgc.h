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
        QString dirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/isocurves2/";
        QVector<SubjectIsoCurves> data = IsoCurveProcessing::readDirectory(dirPath, "d", "*.xml");
        IsoCurveProcessing::selectIsoCurves(data, 0, 5);
        IsoCurveProcessing::stats(data);
        IsoCurveProcessing::sampleIsoCurvePoints(data, 5);
        QVector<Template> rawData = IsoCurveProcessing::generateTemplates(data);
        //QVector<Template> rawData = IsoCurveProcessing::generateEuclDistanceTemplates(data);

        QVector<int> classes;
        QVector<Vector> rawFeatureVectors;
        Template::splitVectorsAndClasses(rawData, rawFeatureVectors, classes);

        int clusterCount = 10;
        QList<QVector<int> > classesInClusters;
        QList<QVector<Vector> > rawVectorsInClusters;
        BioDataProcessing::divideToNClusters(rawFeatureVectors, classes, clusterCount, rawVectorsInClusters, classesInClusters);

        PCA pca(rawVectorsInClusters[0]);
        //PCAExtractor pcaExtractor(pca);
        ZScorePCAExtractor zscorePcaExtractor(pca, rawVectorsInClusters[1]);
        zscorePcaExtractor.serialize("../../test/isocurves/shifted-pca.yml", "../../test/isocurves/shifted-normparams.yml");
        //EuclideanMetric euclMetric;
        CosineMetric cosMetric;

        BatchEvaluationResult batchResult = Evaluation::batch(rawVectorsInClusters, classesInClusters, zscorePcaExtractor, cosMetric, 2);
        qDebug() << batchResult.meanEER << batchResult.stdDevOfEER;
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

    static void evaluateHistogramFeatures()
    {
        QString srcDirPath = "/home/stepo/data/frgc/spring2004/zbin-aligned/depth2";
        QDir srcDir(srcDirPath, "*.png");
        QFileInfoList srcFiles = srcDir.entryInfoList();

        QVector<Template> allTemplates;
        Matrix gaussKernel = KernelGenerator::gaussianKernel(21);
        cv::imshow("kernel", gaussKernel);
        foreach (const QFileInfo &fileInfo, srcFiles)
        {
            ImageGrayscale full = cv::imread(fileInfo.absoluteFilePath().toStdString(), cv::IMREAD_GRAYSCALE);
            cv::filter2D(full, full, CV_8U, gaussKernel);
            cv::imshow("smoothed", full);
            cv::waitKey();
            ImageGrayscale cropped = full(cv::Rect(40, 20, 220, 180));
            HistogramFeatures features(cropped, 6, 6);

            Template t;
            t.subjectID = fileInfo.baseName().split('d')[0].toInt();
            t.featureVector = features.toVector();
            allTemplates << t;

            if (allTemplates.count() == 500) break;
        }

        QVector<int> classes;
        QVector<Vector> vectors;
        Template::splitVectorsAndClasses(allTemplates, vectors, classes);
        PCA pca(vectors);
        ZScorePCAExtractor extractor(pca, vectors);
        CosineMetric cosMetric;
        Evaluation e1(vectors, classes, extractor, cosMetric);
        qDebug() << e1.eer;

        CityblockMetric metric;
        Evaluation e2(allTemplates, metric);
        qDebug() << e2.eer;
    }
};

#endif
