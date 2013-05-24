#ifndef BIODATAPROCESSING_H
#define BIODATAPROCESSING_H

#include <QVector>
#include <QList>
#include <QSet>

#include "template.h"
#include "linalg/common.h"

class BioDataProcessing
{
public:
    static QList<QVector<Template> > divideTemplatesToClusters(QVector<Template> &templates, int subjectsInOneCluster);

    static void divideTemplatesToClusters(QVector<Vector> &vectors, QVector<int> &classMembership, int subjectsInOneCluster,
                       QList<QVector<Vector> > &resultVectors, QList<QVector<int> > &resultClasses);

    static void divideToNClusters(QVector<Matrix> &vectors, QVector<int> &classMembership, int numberOfClusters,
                                  QList<QVector<Matrix> > &resultVectors, QList<QVector<int> > &resultClasses);

    static void divideToNClusters(QVector<Vector> &vectors, QVector<int> &classMembership, int numberOfClusters,
                                  QList<QVector<Vector> > &resultVectors, QList<QVector<int> > &resultClasses);

    static QList<QSet<int> > divideToNClusters(QVector<int> &classMembership, int numberOfClusters);

    static void divideAccordingToUniqueClasses(QVector<Vector> &vectors, QVector<int> &classMembership,
            QList<QSet<int > > &uniqueClassesInClusters,
            QList<QVector<Vector> > &resultVectors, QList<QVector<int> > &resultClasses);
};

#endif // BIODATAPROCESSING_H
