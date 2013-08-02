#ifndef EVALUATEKINECT_H
#define EVALUATEKINECT_H

#include <QDir>
#include <QFileInfoList>
#include <QFileInfo>
#include <QApplication>

#include "facelib/glwidget.h"
#include "facelib/mesh.h"
#include "facelib/surfaceprocessor.h"
#include "linalg/common.h"
#include "biometrics/featureextractor.h"
#include "biometrics/isocurveprocessing.h"
#include "biometrics/evaluation.h"
#include "biometrics/scorelevefusion.h"
#include "biometrics/histogramfeatures.h"
#include "facelib/landmarkdetector.h"
#include "facelib/facealigner.h"
#include "linalg/kernelgenerator.h"
#include "linalg/serialization.h"
#include "linalg/matrixconverter.h"

class EvaluateKinect
{
public:
    static void evaluate()
    {
        //QApplication app(0, 0);
        //GLWidget widget;

        ZScorePCAExtractor curveExtractor("../../test/isocurves/shifted-pca.yml", "../../test/isocurves/shifted-normparams.yml");
        ZScorePCAExtractor depthExtractor("../../test/frgc/depth/pca.yml", "../../test/frgc/depth/normparams.yml");
        cv::Rect depthRoi(50, 30, 200, 180);
        ZScorePassExtractor histExtractor("../../test/frgc/histogram/normparams.yml");
        ZScorePCAExtractor textureExtractor("../../test/frgc/texture/pca.yml", "../../test/frgc/texture/normparams.yml");

        //Mesh referenceFace = Mesh::fromOBJ("../../test/meanForAlign.obj");
        //FaceAligner aligner(referenceFace);

        Matrix smoothKernel = KernelGenerator::gaussianKernel(7);

        QDir dir("../../test/kinect/", "*.bin");
        QFileInfoList binFiles = dir.entryInfoList();
        QVector<int> classes;
        QVector<Vector> curveVectors;
        QVector<Vector> depthVectors;
        QVector<Vector> histVectors;
        QVector<Vector> textureVectors;

        foreach(const QFileInfo &info, binFiles)
        {
            Mesh face = Mesh::fromBIN(info.absoluteFilePath());

            //Mesh frgcFace = Mesh::fromBINZ("/home/stepo/data/frgc/spring2004/zbin-aligned/02463d652.binz");
            //VectorOfIsocurves frgcCurves = Serialization::readVectorOfPointclouds("/home/stepo/data/frgc/spring2004/zbin-aligned/isocurves2/02463d652.xml");

            //aligner.icpAlign(face, 10);
            //face.writeBIN(info.absoluteFilePath());

            MapConverter converter;
            Map depth = SurfaceProcessor::depthmap(face, converter, cv::Point2d(-75, -75), cv::Point2d(75, 75), 2, ZCoord);
            depth.applyFilter(smoothKernel, 3, true);
            Matrix depthMat = depth.toMatrix(0, -75, 0);
            depthMat = depthMat(depthRoi);

            MapConverter histogramConverter;
            Map histogramDepth = SurfaceProcessor::depthmap(face, histogramConverter, cv::Point2d(-60,-30), cv::Point2d(60, 60), 2.0, ZCoord);
            histogramDepth.applyFilter(smoothKernel, 3, true);

            MapConverter textureConverter;
            Map texture = SurfaceProcessor::depthmap(face, textureConverter, cv::Point2d(-50, -30), cv::Point2d(50, 60), 1, Texture_I);
            Matrix textureMat = texture.toMatrix(0, 0, 255);

            /*cv::imshow("test", textureMat);
            cv::waitKey(1);*/

            cv::Point3d center(0,20,0);
            VectorOfIsocurves isocurves;
            for (int distance = 10; distance <= 50; distance += 10)
            {
                VectorOfPoints isocurve = SurfaceProcessor::isoGeodeticCurve(depth, converter, center, distance, 100, 2);
                isocurves << isocurve;
            }

            int id = info.baseName().split('-')[0].toInt();
            classes << id;

            curveVectors << IsoCurveProcessing::generateFeatureVector(isocurves);
            depthVectors << MatrixConverter::matrixToColumnVector(depthMat);
            histVectors << HistogramFeatures(depth, 20, 20).toVector();
            textureVectors << MatrixConverter::matrixToColumnVector(textureMat);

            /*widget.addFace(&face);
            widget.addFace(&referenceFace);
            for (int i = 0; i < isocurves.count(); i++)
            {
                widget.addCurve(isocurves[i]);
                //widget.addCurve(frgcCurves[i]);
            }
            widget.show();
            app.exec();
            exit(0);*/
        }

        CorrelationMetric cor;
        CityblockMetric sad;

        Evaluation curveEval(curveVectors, classes, curveExtractor, cor);
        qDebug() << "isocurves" << curveEval.eer;
        curveEval.outputResults("kinect-curves", 10);

        Evaluation depthEval(depthVectors, classes, depthExtractor, cor);
        qDebug() << "depth" << depthEval.eer;
        depthEval.outputResults("kinect-depth", 10);

        CorrelationWeightedMetric histMetric;
        histMetric.w = Vector::fromFile("../../test/frgc/histogram/selectionWeights");
        Evaluation histEval(histVectors, classes, histExtractor, sad);
        qDebug() << "histogram" << histEval.eer;
        histEval.outputResults("kinect-hist", 10);

        Evaluation textureEval(textureVectors, classes, textureExtractor, cor);
        qDebug() << "texture" << textureEval.eer;
        textureEval.outputResults("kinect-texture", 10);

        QList<QVector<Vector> > joinedVectors;
        joinedVectors << curveVectors << depthVectors << histVectors << textureVectors;
        ScoreLogisticRegressionFusion fusion;
        fusion.addComponent(curveVectors, classes, curveExtractor, cor);
        fusion.addComponent(depthVectors, classes, depthExtractor, cor);
        fusion.addComponent(histVectors, classes, histExtractor, sad);
        fusion.addComponent(textureVectors, classes, textureExtractor, cor);
        fusion.learn();

        Evaluation fusionEval = fusion.evaluate(joinedVectors, classes);
        qDebug() << "fusion" << fusionEval.eer;
        fusionEval.outputResults("kinect-fusion", 10);
    }
};

#endif // EVALUATEKINECT_H
