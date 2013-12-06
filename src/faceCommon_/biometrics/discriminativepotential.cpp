#include "discriminativepotential.h"

#include <QDebug>
#include <QMap>

#include <cassert>

#include "linalg/vector.h"
#include "linalg/metrics.h"

DiscriminativePotential::DiscriminativePotential(const QVector<Template> &templates)
{
    int n = templates.count();
    assert(n > 0);

    int m = templates.at(0).featureVector.rows;
    assert(m > 0);

    QVector<Template> normalizedTemplates = Template::clone(templates);
    Template::zScoreNorm(normalizedTemplates);

    // Calculate DP
    // for each component
    scores = Vector(m);
    for (int i = 0; i < m; i++)
    {
        QMap<int, QVector<double> > valuesForSubj;

        // for each template
        for (int j = 0; j < n; j++)
        {
            const Template &t = normalizedTemplates.at(j);
            double val = t.featureVector(i);

            // nan guard
            if (val != val)
            {
            	qDebug() << "subj" << t.subjectID << "index" << i << "nan";
            }

            if(valuesForSubj.contains(t.subjectID))
            {
                valuesForSubj[t.subjectID].append(val);
            }
            else
            {
                QVector<double> values;
                values.append(val);
                valuesForSubj[t.subjectID] = values;
            }
        }

        // DP for specific FV component i:
        // dp(i) = stdDev of class means - mean of std deviation within each class
        QList<int> uniqueSubjects = valuesForSubj.uniqueKeys();
        QVector<double> classMeans;
        QVector<double> stdDeviationForClass;
        foreach(int subjID, uniqueSubjects)
        {
        	if (valuesForSubj[subjID].count() <= 1)
        	{
                qDebug() << "Warning, subject" << subjID << "has only one scan!";
                continue;
        	}

            Vector valuesForSubjVec(valuesForSubj[subjID]);
            classMeans << valuesForSubjVec.meanValue();
            stdDeviationForClass << valuesForSubjVec.stdDeviation();
        }

        Vector classMeansVec(classMeans);
        Vector stdDeviationForClassVec(stdDeviationForClass);
        scores(i) = classMeansVec.stdDeviation() - stdDeviationForClassVec.meanValue();
    }

    minScore = scores.minValue();
    maxScore = scores.maxValue();
}

