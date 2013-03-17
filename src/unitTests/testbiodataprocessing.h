#ifndef TESTBIODATAPROCESSING_H
#define TESTBIODATAPROCESSING_H

#include <QVector>
#include <QList>
#include <QDebug>

#include "linalg/common.h"
#include "linalg/loader.h"
#include "biometrics/biodataprocessing.h"

class TestBioDataProcessing
{
public:
    static void testDivideVectors()
    {
        QVector<Vector> allVectors;
        QVector<int> allClasses;
        QString path("/media/frgc/frgc-norm-iterative/big-index");
        Loader::loadImages(path, allVectors, &allClasses, "*.png");

        QList<QVector<int> > classesInClusters;
        QList<QVector<Vector> > vectorsInClusters;
        BioDataProcessing::divide(allVectors, allClasses, 25, vectorsInClusters, classesInClusters);

        int n = classesInClusters.count();
        for (int i = 0; i < n; i++)
        {
            qDebug() << i << classesInClusters[i].count() << vectorsInClusters[i].count();
        }
    }
};

#endif // TESTBIODATAPROCESSING_H
