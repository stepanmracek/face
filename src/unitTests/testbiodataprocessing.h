#ifndef TESTBIODATAPROCESSING_H
#define TESTBIODATAPROCESSING_H

#include <QVector>
#include <QList>
#include <QDebug>
#include <QSet>

#include "linalg/common.h"
#include "linalg/loader.h"
#include "biometrics/biodataprocessing.h"
#include "linalg/serialization.h"
#include "biometrics/isocurveprocessing.h"

class TestBioDataProcessing
{
    static QString dataPath() { return "/home/stepo/data/frgc/spring2004/zbin-aligned/depth1"; }

public:
    static void testDivideTemplatesToClusters()
    {
        QVector<Vector> allVectors;
        QVector<int> allClasses;
        Loader::loadImages(dataPath(), allVectors, &allClasses, "*.png", "d");
        QVector<Template> allTemplates = Template::joinVectorsAndClasses(allVectors, allClasses);

        qDebug() << "dividing...";
        QList<QVector<Template> > templatesInClusters = BioDataProcessing::divideTemplatesToClusters(allTemplates, 20);
        QSet<int> allPreviousClasses;
        foreach (const QVector<Template> &cluster, templatesInClusters)
        {
            QSet<int> currentClasses;
            foreach (const Template &t, cluster)
            {
                currentClasses << t.subjectID;
                assert(!allPreviousClasses.contains(t.subjectID));
            }
            qDebug() << "classes: " << currentClasses.count() << "scans:" << cluster.count();

            allPreviousClasses += currentClasses;
        }
    }

    static void testDivideVectorsToClusters()
    {
        QVector<Vector> allVectors;
        QVector<int> allClasses;
        Loader::loadImages(dataPath(), allVectors, &allClasses, "*.png", "d");

        qDebug() << "dividing...";
        QList<QVector<Vector> > vectorsInClusters;
        QList<QVector<int> > classesInClusters;
        BioDataProcessing::divideVectorsToClusters(allVectors, allClasses, 20, vectorsInClusters, classesInClusters);

        QSet<int> allPreviousClasses;
        foreach (const QVector<int> &cluster, classesInClusters)
        {
            QSet<int> currentClasses;
            foreach (int c, cluster)
            {
                currentClasses << c;
                assert(!allPreviousClasses.contains(c));
            }
            qDebug() << "classes: " << currentClasses.count() << "scans:" << cluster.count();

            allPreviousClasses += currentClasses;
        }
    }

    static void testDivideToNClusters()
    {
        QVector<Vector> allVectors;
        QVector<int> allClasses;
        Loader::loadImages(dataPath(), allVectors, &allClasses, "*.png", "d");

        qDebug() << "dividing...";
        QList<QVector<Vector> > vectorsInClusters;
        QList<QVector<int> > classesInClusters;
        BioDataProcessing::divideToNClusters(allVectors, allClasses, 10, vectorsInClusters, classesInClusters);

    }
};

#endif // TESTBIODATAPROCESSING_H
