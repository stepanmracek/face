#include "biodataprocessing.h"

#include <cassert>
//#include <ctime>

/**
 * @brief Divides templates into desired count of clusters. There will be exactly the same count of individual
 * subjects in each cluster, excluding the last one, where the remain subjects will be stored. The exact
 * amount of scans within each cluster may vary since the number of scans per one subject may vary.
 * *NOTE*: the method expects that the input templates are sorted (e.g. [sub1, sub1, sub1, sub2, sub2, sub3,...])
 * @param templates Input sorted templates
 * @param subjectsInOneCluster Desired count of subjects in each resulting cluster
 * @return Returns templates divided into clusters
 */
QList< QVector<Template> > BioDataProcessing::divideTemplatesToClusters(QVector<Template> &templates, int subjectsInOneCluster)
{
    assert(subjectsInOneCluster > 1);
    QSet<int> currentClusterClasses;
    QList< QVector<Template> > result;
    int currentClusterIndex = 0;

    //init
    QVector<Template> ts;
    result.append(ts);

    // Iterate through the input templates
    int n = templates.count();
    for (int i = 0; i < n; i++)
    {
        Template &t = templates[i];
        // If the current subject is already stored in current cluster
        if (currentClusterClasses.contains(t.subjectID))
        {
            // ok, add it
            result[currentClusterIndex].append(t);
        }
        else
        {
            // nope. We have to check count of subjects in current cluster
            if (currentClusterClasses.count() >= subjectsInOneCluster)
            {
                // Subjects count exceeded, we have to crate new cluster
                currentClusterClasses.clear();
                QVector<Template> ts;
                result.append(ts);
                currentClusterIndex++;

                // We can add curret subject to new  cluster
                currentClusterClasses << t.subjectID;
                result[currentClusterIndex].append(t);
            }
            else
            {
                // We can add curret subject to current cluster
                currentClusterClasses << t.subjectID;
                result[currentClusterIndex].append(t);
            }
        }
    }

    return result;
}

void BioDataProcessing::divideVectorsToClusters(QVector<Vector> &vectors, QVector<int> &classMembership, int subjectsInOneCluster,
                                                QList<QVector<Vector> > &resultVectors, QList<QVector<int> > &resultClasses)
{
    int n = vectors.count();
    assert(n == classMembership.count());
    assert(subjectsInOneCluster > 1);
    QSet<int> currentClusterClasses;
    int currentResultIndex = 0;

    //init
    QVector<Vector> ms;
    QVector<int> cs;
    resultVectors.append(ms);
    resultClasses.append(cs);

    for (int i = 0; i < n; i++)
    {
        Vector &v = vectors[i];
        int c = classMembership[i];
        if (currentClusterClasses.contains(c))
        {
            // ok, add it
            resultVectors[currentResultIndex].append(v);
            resultClasses[currentResultIndex].append(c);
        }
        else
        {
            if (currentClusterClasses.count() >= subjectsInOneCluster)
            {
                // new cluster
                currentClusterClasses.clear();
                QVector<Vector> ms;
                QVector<int> cs;
                resultVectors.append(ms);
                resultClasses.append(cs);
                currentResultIndex++;
                i--;
            }
            else
            {
                // create new class and add the template
                currentClusterClasses << c;
                resultVectors[currentResultIndex].append(v);
                resultClasses[currentResultIndex].append(c);
            }
        }
    }
}

void BioDataProcessing::divideToNClusters(
		QVector<Matrix> &vectors, QVector<int> &classMembership,
		int numberOfClusters,
		QList<QVector<Matrix> > &resultVectors, QList<QVector<int> > &resultClasses)
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

    QMultiMap<int, Matrix> classToVectors;
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
        QVector<Matrix> m;
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
        QList<Matrix> curClassVectors = classToVectors.values(key);
        for (int j = 0; j < curClassVectors.count(); j++)
        {
            resultClasses[currentCluster].append(key);
            resultVectors[currentCluster].append(curClassVectors[j]);
        }
    }
}

void BioDataProcessing::divideToNClusters(QVector<Vector> &vectors, QVector<int> &classMembership, int numberOfClusters,
                                          QList<QVector<Vector> > &resultVectors, QList<QVector<int> > &resultClasses)
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

    QMultiMap<int, Vector> classToVectors;
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
        QVector<Vector> m;
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
        QList<Vector> curClassVectors = classToVectors.values(key);
        for (int j = 0; j < curClassVectors.count(); j++)
        {
            resultClasses[currentCluster].append(key);
            resultVectors[currentCluster].append(curClassVectors[j]);
        }
    }
}

QList<QSet<int> > BioDataProcessing::divideToNClusters(
		QVector<int> &classMembership,
		int numberOfClusters)
{
	assert(numberOfClusters > 1);

	static bool randomized = false;
	if (!randomized)
	{
		//qDebug() << "randomizing";
		qsrand(time(NULL));
		randomized = true;
	}

    QList<QSet<int> > uniqueClassesInClusters;

    QList<int> tmp1Keys = QList<int>::fromVector(classMembership);
    QSet<int> tmp2Keys = QSet<int>::fromList(tmp1Keys);
    QList<int> permutatedUniqueClasses = QList<int>::fromSet(tmp2Keys);
    for (int i = permutatedUniqueClasses.count()-1; i > 0; --i)
    {
        int randIndex = qrand() % (i+1);
        qSwap(permutatedUniqueClasses[i], permutatedUniqueClasses[randIndex]);
    }

    int countPerCluster = permutatedUniqueClasses.count()/numberOfClusters;
    for (int i = 0; i < numberOfClusters; i++)
    {
    	QSet<int> c;
        uniqueClassesInClusters.append(c);
    }

    int currentCluster = 0;
    for (int i = 0; i < permutatedUniqueClasses.count(); i++)
    {
        QSet<int> &currentClasses = uniqueClassesInClusters[currentCluster];
        int currentCount = currentClasses.count();
        if (currentCluster != (numberOfClusters-1) && currentCount >= countPerCluster)
            currentCluster++;

        int key = permutatedUniqueClasses[i];
        uniqueClassesInClusters[currentCluster].insert(key);
    }

    return uniqueClassesInClusters;
}

void BioDataProcessing::divideAccordingToUniqueClasses(QVector<Vector> &vectors, QVector<int> &classMembership,
        QList<QSet<int > > &uniqueClassesInClusters,
        QList<QVector<Vector> > &resultVectors, QList<QVector<int> > &resultClasses)
{
	int clustersCount = uniqueClassesInClusters.count();
	for (int curCluster = 0; curCluster < clustersCount; curCluster++)
	{
		QVector<int> curClusterClasses;
        QVector<Vector> curClusterVectors;
		foreach(int subjID, uniqueClassesInClusters[curCluster])
		{
			for (int i = 0; i < classMembership.count(); i++)
			{
				if (subjID == classMembership[i])
				{
					curClusterClasses << subjID;
					curClusterVectors << vectors[i];
				}
			}
		}
		resultClasses << curClusterClasses;
		resultVectors << curClusterVectors;
	}
}
