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

    static void divideVectorsToClusters(QVector<Vector> &vectors, QVector<int> &classMembership, int subjectsInOneCluster,
                                        QList<QVector<Vector> > &resultVectors, QList<QVector<int> > &resultClasses);

    /*static void divideToNClusters(QVector<Matrix> &vectors, QVector<int> &classMembership, int numberOfClusters,
                                  QList<QVector<Matrix> > &resultVectors, QList<QVector<int> > &resultClasses);*/

    template <class T>
    /**
     * @brief Divides input data into desired count of clusters
     * @param vectors Input vectors
     * @param classMembership Input classes
     * @param numberOfClusters Desired count of resulting clusters
     * @param resultVectors
     * @param resultClasses
     */
    static void divideToNClusters(QVector<T> &vectors, QVector<int> &classMembership, int numberOfClusters,
                                  QList<QVector<T> > &resultVectors, QList<QVector<int> > &resultClasses)
    {
        static bool randomized = false;
        if (!randomized)
        {
            //qDebug() << "randomizing";
            qsrand(time(NULL));
            randomized = true;
        }

        int n = vectors.count();
        assert(n == classMembership.count());
        assert(numberOfClusters > 1);

        QMultiMap<int, T> classToVectors;
        for (int i = 0; i < n; i++)
            classToVectors.insertMulti(classMembership[i], vectors[i]);

        QList<int> keys = classToVectors.uniqueKeys();
        for (int i = keys.count()-1; i > 0; --i)
        {
            int randIndex = qrand() % (i+1);
            qSwap(keys[i], keys[randIndex]);
        }

        int countPerCluster = keys.count()/numberOfClusters;
        int currentCluster = 0;
        for (int i = 0; i < numberOfClusters; i++)
        {
            QVector<T> m;
            QVector<int> c;
            resultVectors.append(m);
            resultClasses.append(c);
        }

        for (int i = 0; i < keys.count(); i++)
        {
            QList<int> tmpList = QList<int>::fromVector(resultClasses[currentCluster]);
            QSet<int> currentClasses = QSet<int>::fromList(tmpList);
            int currentCount = currentClasses.count();
            if (currentCluster != (numberOfClusters-1) && currentCount >= countPerCluster)
                currentCluster++;

            int key = keys[i];
            QList<T> curClassVectors = classToVectors.values(key);
            for (int j = 0; j < curClassVectors.count(); j++)
            {
                resultClasses[currentCluster].append(key);
                resultVectors[currentCluster].append(curClassVectors[j]);
            }
        }
    }

    /*static QList<QSet<int> > divideToNClusters(QVector<int> &classMembership, int numberOfClusters);

    static void divideAccordingToUniqueClasses(QVector<Vector> &vectors, QVector<int> &classMembership,
            QList<QSet<int > > &uniqueClassesInClusters,
            QList<QVector<Vector> > &resultVectors, QList<QVector<int> > &resultClasses);*/
};

#endif // BIODATAPROCESSING_H
