#include "discriminativepotential.h"

#include <QDebug>
#include <QMap>

#include <cassert>

#include "linalg/vector.h"
#include "linalg/metrics.h"

DiscriminativePotential::DiscriminativePotential(QVector<Template> &templates)
{
    //qDebug() << "Discriminative potential evaluation...";

    int n = templates.count();
    assert(n > 0);

    int m = templates.at(0).featureVector.rows;
    assert(m > 0);

    /*for (int i = 0; i < m; i++)
    {
        minValues.append(1e300);
        maxValues.append(-1e300);
    }*/

    QVector<Template> normalizedTemplates = Template::clone(templates);
    Template::zScoreNorm(normalizedTemplates);

    /*// get min and max values
    for (int i = 0; i < n; i++)
    {
       const Template &t = templates.at(i);
       for (int j = 0; j < m; j++)
       {
           double val = t.featureVector(j);
           if (val > maxValues[j])
               maxValues[j] = val;
           if (val < minValues[j])
           {
               minValues[j] = val;
           }
       }
    }*/

    /*// normalize to range [0..1]
    for (int i = 0; i < n; i++)
    {
       const Template &t = templates.at(i);
       Template normalized;
       normalized.subjectID = t.subjectID;
       normalized.featureVector = Matrix::zeros(m, 1, CV_64F);
       for (int j = 0; j < m; j++)
       {
           double val = t.featureVector(j);
           normalized.featureVector(j) = (val-minValues[j])/(maxValues[j] - minValues[j]);
       }
       normalizedTemplates.append(normalized);
    }*/

    // Calculate DP
    // for each component
    for (int i = 0; i < m; i++)
    {
        //qDebug() << " Evaluating component" << (i+1) << "/" << m;
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
        	}

            Vector valuesForSubjVec(valuesForSubj[subjID]);
            classMeans << valuesForSubjVec.meanValue();
            stdDeviationForClass << valuesForSubjVec.stdDeviation();
        }

        Vector classMeansVec(classMeans);
        Vector stdDeviationForClassVec(stdDeviationForClass);
        double s = classMeansVec.stdDeviation() - stdDeviationForClassVec.meanValue();
        scores << s;

        /*QList<QVector<double> > values = valuesForSubj.values();
        QVector<double> rangesForSubj;
        QVector<double> meansForSubj;
        for (int j = 0; j < values.count(); j++)
        {
            double r = Vector::maxValue(values[j]) - Vector::minValue(values[j]);
            rangesForSubj.append(r);

            meansForSubj.append(Vector::meanValue(values[j]));
        }

        // create histogram
        QVector<double> hist(10);
        for (int j = 0; j < values.count(); j++)
        {
            QVector<double> &curSubjVals = values[j];

            for (int k = 0; k < curSubjVals.count(); k++)
            {
                double value = curSubjVals[k];
                int bin = (int)(value * 10);
                if (bin == 10) bin = 9;

                hist[bin] += 1.0/n;
            }
        }

        Matrix uniform = Matrix::ones(10, 1, CV_64F);
        uniform *= 0.1;
        Matrix histMat = Vector::fromQVector(hist);
        Matrix distMat = (uniform-histMat).t() * (uniform-histMat);
        double dist = distMat(0);

        double mu = Vector::meanValue(rangesForSubj);
        double s = Vector::stdDeviation(rangesForSubj);
        double max = Vector::maxValue(rangesForSubj);

        double score = 1 - (mu + s + max) - dist;
        scores.append(score);*/
    }

    minScore = scores.minValue();
    maxScore = scores.maxValue();

    //qDebug() << "...done";
}

