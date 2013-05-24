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
    static void testDivide()
    {
        QVector<Vector> allVectors;
        QVector<int> allClasses;
        Loader::loadImages(dataPath(), allVectors, &allClasses, "*.png", "d");
        QVector<Template> allTemplates = Template::joinVectorsAndClasses(allVectors, allClasses);

        qDebug() << "dividing...";
        QList<QVector<Template> > templatesInClusters = BioDataProcessing::divide(allTemplates, 20);
        foreach (const QVector<Template> &cluster, templatesInClusters)
        {
            QSet<int> uniqueClasses;
            foreach (const Template &t, cluster)
            {
                uniqueClasses << t.subjectID;
            }
            qDebug() << uniqueClasses.count();
        }
    }
};

#endif // TESTBIODATAPROCESSING_H
