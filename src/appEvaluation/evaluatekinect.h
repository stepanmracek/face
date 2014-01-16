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
#include "linalg/loader.h"
#include "linalg/matrixconverter.h"
#include "biometrics/facetemplate.h"

class EvaluateKinect
{
public:

    static void evaluateReferenceDistances()
    {
        FaceClassifier faceClassifier("../../test/kinect/classifiers2/");

        QMap<int, Face3DTemplate *> references;

        QVector<QString> templateFiles = Loader::listFiles("../../test/kinect/", "*.yml", Loader::AbsoluteFull);
        foreach(const QString &path, templateFiles)
        {
            int id = QFileInfo(path).baseName().split("-")[0].toInt();
            Face3DTemplate *t = new Face3DTemplate(id, path, faceClassifier);
            references.insertMulti(id, t);
        }

        foreach (int id, references.uniqueKeys())
        {
            qDebug() << id;
            QList<Face3DTemplate *> ref = references.values(id);
            foreach (const Face3DTemplate *probe, ref)
            {
                qDebug() << "  " << faceClassifier.compare(ref, probe, FaceClassifier::CompareMeanDistance);
            }
        }
    }

    static void evaluateRefeference()
    {
        FaceClassifier faceClassifier("../../test/kinect/classifiers/");

        QHash<int, Face3DTemplate *> references;
        QVector<Face3DTemplate *> testTemplates;

        QVector<QString> templateFiles = Loader::listFiles("../../test/kinect/", "*.yml", Loader::AbsoluteFull);
        foreach(const QString &path, templateFiles)
        {
            int id = QFileInfo(path).baseName().split("-")[0].toInt();
            Face3DTemplate *t = new Face3DTemplate(id, path, faceClassifier);

            if (!references.contains(id) || references.values(id).count() < 2)
            {
                references.insertMulti(id, t);
            }
            else
            {
                testTemplates << t;
            }
        }

        Evaluation e = faceClassifier.evaluate(references, testTemplates, FaceClassifier::CompareMeanDistance);
        qDebug() << e.eer; // << e.fnmrAtFmr(0.01) << e.fnmrAtFmr(0.001) << e.fnmrAtFmr(0.0001);
        qDebug() << e.maxSameDistance << e.minDifferentDistance;
        //e.outputResults("kinect", 10);
    }

    static void evaluateSimple()
    {
        FaceClassifier faceClassifier("../../test/kinect/classifiers/");
        QVector<Face3DTemplate*> templates;

        QVector<QString> templateFiles = Loader::listFiles("../../test/kinect/", "*.yml", Loader::AbsoluteFull);
        foreach(const QString &path, templateFiles)
        {
            int id = QFileInfo(path).baseName().split("-")[0].toInt();
            templates << new Face3DTemplate(id, path, faceClassifier);
        }

        Evaluation e = faceClassifier.evaluate(templates);
        qDebug() << e.eer;

        foreach (Face3DTemplate *t, templates)
        {
            delete t;
        }
        //e.outputResults("kinect", 15);
    }

    static void createTemplates()
    {
        FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj", false));
        FaceClassifier faceClassifier("../../test/kinect/classifiers2/");

        QVector<QString> binFiles = Loader::listFiles("../../test/kinect/", "*.bin", Loader::AbsoluteFull);
        foreach(const QString &path, binFiles)
        {
            QString baseName = QFileInfo(path).baseName();
            int id = baseName.split("-")[0].toInt();
            Mesh face = Mesh::fromBIN(path);
            aligner.icpAlign(face, 10, FaceAligner::NoseTipDetection);

            Face3DTemplate t(id, face, faceClassifier);
            t.serialize("../../test/kinect/" + baseName + ".yml.gz", faceClassifier);
        }
    }

    static void learnFromFrgc()
    {
        FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj", false));
        FaceClassifier faceClassifier("../../test/frgc/classifiers/");
        QVector<Face3DTemplate*> templates;

        QVector<QString> binFiles = Loader::listFiles("../../test/softKinetic/01/", "*.binz", Loader::AbsoluteFull);
        foreach(const QString &path, binFiles)
        {
            int id = QFileInfo(path).baseName().split("-")[0].toInt();
            Mesh face = Mesh::fromBINZ(path);
            SurfaceProcessor::zsmooth(face, 0.5, 5);
            aligner.icpAlign(face, 10, FaceAligner::NoseTipDetection);
            templates << new Face3DTemplate(id, face, faceClassifier);


        }

        FaceClassifier newClassifier;
        faceClassifier.relearnFinalFusion(templates, newClassifier, true);
        newClassifier.serialize("../../test/kinect/classifiers2");
    }

    static void evaluateKinect()
    {
        FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj", false));
        FaceClassifier faceClassifier("../../test/kinect/classifiers2/");
        QVector<Face3DTemplate*> templates;

        QVector<QString> binFiles = Loader::listFiles("../../test/kinect/", "*.bin", Loader::AbsoluteFull);
        foreach(const QString &path, binFiles)
        {
            int id = QFileInfo(path).baseName().split("-")[0].toInt();
            Mesh face = Mesh::fromBIN(path, true);
            aligner.icpAlign(face, 10, FaceAligner::NoseTipDetection);
            templates << new Face3DTemplate(id, face, faceClassifier);
            templates.last()->serialize("../../test/kinect/" + QFileInfo(path).baseName() + ".xml", faceClassifier);
        }

        Evaluation eval = faceClassifier.evaluate(templates);
        qDebug() << eval.eer;
        //eval.outputResults("kinect", 10);
    }

    static void evaluateSerializedKinect()
    {
        FaceClassifier faceClassifier("../../test/kinect/classifiers2/");
        QVector<Face3DTemplate*> templates;

        QVector<QString> binFiles = Loader::listFiles("../../test/kinect/", "*.xml", Loader::AbsoluteFull);
        foreach(const QString &path, binFiles)
        {
            int id = QFileInfo(path).baseName().split("-")[0].toInt();
            templates << new Face3DTemplate(id, path, faceClassifier);
        }

        Evaluation eval = faceClassifier.evaluate(templates);
        qDebug() << eval.eer;
    }

};
#endif // EVALUATEKINECT_H
