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
        QVector<Face::LinAlg::Vector> allVectors;
        QVector<int> allClasses;
        Face::LinAlg::Loader::loadImages(dataPath(), allVectors, &allClasses, "*.png", "d");
        QVector<Face::Biometrics::Template> allTemplates = Face::Biometrics::Template::joinVectorsAndClasses(allVectors, allClasses);

        qDebug() << "dividing...";
        QList<QVector<Face::Biometrics::Template> > templatesInClusters = Face::Biometrics::BioDataProcessing::divideTemplatesToClusters(allTemplates, 20);
        QSet<int> allPreviousClasses;
        foreach (const QVector<Face::Biometrics::Template> &cluster, templatesInClusters)
        {
            QSet<int> currentClasses;
            foreach (const Face::Biometrics::Template &t, cluster)
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
        QVector<Face::LinAlg::Vector> allVectors;
        QVector<int> allClasses;
        Face::LinAlg::Loader::loadImages(dataPath(), allVectors, &allClasses, "*.png", "d");

        qDebug() << "dividing...";
        QList<QVector<Face::LinAlg::Vector> > vectorsInClusters;
        QList<QVector<int> > classesInClusters;
        Face::Biometrics::BioDataProcessing::divideVectorsToClusters(allVectors, allClasses, 20, vectorsInClusters, classesInClusters);

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
        QVector<Face::LinAlg::Vector> allVectors;
        QVector<int> allClasses;
        Face::LinAlg::Loader::loadImages(dataPath(), allVectors, &allClasses, "*.png", "d");

        qDebug() << "dividing...";
        QList<QVector<Face::LinAlg::Vector> > vectorsInClusters;
        QList<QVector<int> > classesInClusters;
        Face::Biometrics::BioDataProcessing::divideToNClusters(allVectors, allClasses, 10, vectorsInClusters, classesInClusters);

        QSet<int> allPreviousClasses;
        for (int i = 0; i < 10; i++)
        {
            assert(vectorsInClusters[i].count() == classesInClusters[i].count());

            QSet<int> uniqueClassesInCluster = QSet<int>::fromList(classesInClusters[i].toList());
            qDebug() << (i+1) << "classes:" << uniqueClassesInCluster.count() << "total vectors:" << vectorsInClusters[i].count();

            foreach (int c, uniqueClassesInCluster)
            {
                assert(!allPreviousClasses.contains(c));
            }
            allPreviousClasses += uniqueClassesInCluster;
        }
    }
};

#endif // TESTBIODATAPROCESSING_H
