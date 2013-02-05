#include "biodataprocessing.h"

#include <cassert>
//#include <ctime>

QList< QVector<Template> > BioDataProcessing::divide(QVector<Template> &templates, int subjectsInOneCluster)
{
    assert(subjectsInOneCluster > 1);
    QSet<int> currentClusterClasses;
    QList< QVector<Template> > result;
    int currentResultIndex = 0;

    //init
    QVector<Template> ts;
    result.append(ts);

    int n = templates.count();
    for (int i = 0; i < n; i++)
    {
        Template &t = templates[i];
        if (currentClusterClasses.contains(t.subjectID))
        {
            // ok, add it
            result[currentResultIndex].append(t);
        }
        else
        {
            if (currentClusterClasses.count() >= subjectsInOneCluster)
            {
                // new cluster
                currentClusterClasses.clear();
                QVector<Template> ts;
                result.append(ts);
                currentResultIndex++;
                i--;
            }
            else
            {
                // create new class and add the template
                currentClusterClasses << t.subjectID;
                result[currentResultIndex].append(t);
            }
        }
    }

    return result;
}

void BioDataProcessing::divide(QVector<Matrix> &vectors, QVector<int> &classMembership, int subjectsInOneCluster,
                               QList<QVector<Matrix> > &resultVectors, QList<QVector<int> > &resultClasses)
{
    int n = vectors.count();
    assert(n == classMembership.count());
    assert(subjectsInOneCluster > 1);
    QSet<int> currentClusterClasses;
    int currentResultIndex = 0;

    //init
    QVector<Matrix> ms;
    QVector<int> cs;
    resultVectors.append(ms);
    resultClasses.append(cs);

    for (int i = 0; i < n; i++)
    {
        Matrix &v = vectors[i];
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
                QVector<Matrix> ms;
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

void BioDataProcessing::divideAccordingToUniqueClasses(
		QVector<Matrix> &vectors, QVector<int> &classMembership,
		QList<QSet<int > > &uniqueClassesInClusters,
		QList<QVector<Matrix> > &resultVectors, QList<QVector<int> > &resultClasses)
{
	int clustersCount = uniqueClassesInClusters.count();
	for (int curCluster = 0; curCluster < clustersCount; curCluster++)
	{
		QVector<int> curClusterClasses;
		QVector<Matrix> curClusterVectors;
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
