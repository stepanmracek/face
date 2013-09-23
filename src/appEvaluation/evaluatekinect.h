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
        FaceClassifier faceClassifier("../../test/kinect/classifiers/");

        QMap<int, FaceTemplate *> references;

        QVector<QString> templateFiles = Loader::listFiles("../../test/kinect/", "*.yml", AbsoluteFull);
        foreach(const QString &path, templateFiles)
        {
            int id = QFileInfo(path).baseName().split("-")[0].toInt();
            FaceTemplate *t = new FaceTemplate(id, path, faceClassifier);
            references.insertMulti(id, t);
        }

        foreach (int id, references.uniqueKeys())
        {
            qDebug() << id;
            QList<FaceTemplate *> ref = references.values(id);
            foreach (const FaceTemplate *probe, ref)
            {
                qDebug() << "  " << faceClassifier.compare(ref, probe);
            }
        }
    }

    static void evaluateRefeference()
    {
        FaceClassifier faceClassifier("../../test/kinect/classifiers/");

        QHash<int, FaceTemplate *> references;
        QVector<FaceTemplate *> testTemplates;

        QVector<QString> templateFiles = Loader::listFiles("../../test/kinect/", "*.yml", AbsoluteFull);
        foreach(const QString &path, templateFiles)
        {
            int id = QFileInfo(path).baseName().split("-")[0].toInt();
            FaceTemplate *t = new FaceTemplate(id, path, faceClassifier);

            if (!references.contains(id) || references.values(id).count() < 4)
            {
                references.insertMulti(id, t);
            }
            else
            {
                testTemplates << t;
            }
        }

        Evaluation e = faceClassifier.evaluate(references, testTemplates);
        qDebug() << e.eer << e.maxSameDistance << e.minDifferentDistance;
        e.outputResults("kinect", 10);
    }

    static void evaluateSimple()
    {
        FaceClassifier faceClassifier("../../test/kinect/classifiers/");
        QVector<FaceTemplate*> templates;

        QVector<QString> templateFiles = Loader::listFiles("../../test/kinect/", "*.yml", AbsoluteFull);
        foreach(const QString &path, templateFiles)
        {
            int id = QFileInfo(path).baseName().split("-")[0].toInt();
            templates << new FaceTemplate(id, path, faceClassifier);
        }

        Evaluation e = faceClassifier.evaluate(templates);
        qDebug() << e.eer;
        e.outputResults("kinect", 15);
    }

    static void createTemplates()
    {
        FaceClassifier faceClassifier("../../test/kinect/classifiers/");

        QVector<QString> binFiles = Loader::listFiles("../../test/kinect/", "*.bin", AbsoluteFull);
        foreach(const QString &path, binFiles)
        {
            QString baseName = QFileInfo(path).baseName();
            int id = baseName.split("-")[0].toInt();
            Mesh face = Mesh::fromBIN(path);
            FaceTemplate t(id, face, faceClassifier);
            t.serialize("../../test/kinect/" + baseName + ".yml", faceClassifier);
        }

    }

    static void learnFromFrgc()
    {
        FaceClassifier faceClassifier("../../test/frgc/classifiers/");
        QVector<FaceTemplate> templates;

        QVector<QString> binFiles = Loader::listFiles("../../test/kinect/", "*.bin", AbsoluteFull);
        foreach(const QString &path, binFiles)
        {
            int id = QFileInfo(path).baseName().split("-")[0].toInt();
            Mesh face = Mesh::fromBIN(path);
            templates << FaceTemplate(id, face, faceClassifier);
        }

        ScoreSVMFusion newFusion = faceClassifier.relearnFinalFusion(templates);
        faceClassifier.serialize("../../test/kinect/classifiers");
        newFusion.serialize("../../test/kinect/classifiers/final");
    }

};
#endif // EVALUATEKINECT_H
