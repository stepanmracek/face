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
#include "facelib/landmarkdetector.h"
#include "facelib/facealigner.h"
#include "linalg/kernelgenerator.h"
#include "linalg/serialization.h"

class EvaluateKinect
{
public:
    static void isoCurves()
    {
        ZScorePCAExtractor extractor("../../test/isocurves/shifted-pca.yml", "../../test/isocurves/shifted-normparams.yml");

        //Mesh referenceFace = Mesh::fromOBJ("../../test/meanForAlign.obj");
        //FaceAligner aligner(referenceFace);

        Matrix smoothKernel = KernelGenerator::gaussianKernel(7);

        QDir dir("../../test/kinect/", "*.bin");
        QFileInfoList binFiles = dir.entryInfoList();
        QVector<Template> templates;
        //cv::namedWindow("test");
        foreach(const QFileInfo &info, binFiles)
        {
            Mesh face = Mesh::fromBIN(info.absoluteFilePath());

            //Mesh frgcFace = Mesh::fromBINZ("/home/stepo/data/frgc/spring2004/zbin-aligned/02463d652.binz");
            //VectorOfIsocurves frgcCurves = Serialization::readVectorOfPointclouds("/home/stepo/data/frgc/spring2004/zbin-aligned/isocurves2/02463d652.xml");

            //aligner.icpAlign(face, 10);
            //face.writeBIN(info.absoluteFilePath());

            MapConverter converter;
            Map texture = SurfaceProcessor::depthmap(face, converter, cv::Point2d(-50,-50), cv::Point2d(50,50), 1, Texture_I);
            Map depth = SurfaceProcessor::depthmap(face, converter, 2, ZCoord);
            depth.applyFilter(smoothKernel, 3, true);

            cv::waitKey(1);
            cv::imshow("test",texture.toMatrix());
            cv::waitKey(1);

            cv::Point3d center(0,20,0);
            VectorOfIsocurves isocurves;
            for (int distance = 10; distance <= 50; distance += 10)
            {
                VectorOfPoints isocurve = SurfaceProcessor::isoGeodeticCurve(depth, converter, center, distance, 100, 2);
                isocurves << isocurve;
            }
            IsoCurveProcessing::sampleIsoCurvePoints(isocurves, 5);
            Vector rawVector = IsoCurveProcessing::generateFeatureVector(isocurves);

            Template t;
            t.subjectID = info.baseName().split('-')[0].toInt();
            t.featureVector = extractor.extract(rawVector);
            templates << t;

            /*QApplication app(0, 0);
            GLWidget widget;
            widget.addFace(&face);
            widget.addFace(&frgcFace);
            for (int i = 0; i < isocurves.count(); i++)
            {
                widget.addCurve(frgcCurves[i]);
                widget.addCurve(isocurves[i]);
            }
            widget.show();
            app.exec();
            exit(0);*/
        }

        CosineMetric cos;
        Evaluation e(templates, cos, true);
        qDebug() << e.eer;
    }
};

#endif // EVALUATEKINECT_H
