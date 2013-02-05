#include "template.h"

#include <QMap>
#include <QDir>

#include "linalg/loader.h"

QVector<Template> Template::loadTemplates(const QString &dirPath, const QString &classSeparator)
{
    QVector<Template> templates;
    QStringList filenames = Loader::listFiles(dirPath, "*");
    int n = filenames.count();
    assert(n > 0);

    for (int i = 0; i < n; i++)
    {
        QString path(dirPath + QDir::separator() + filenames[i]);
        Matrix featureVector = Vector::fromFile(path);

        int indexOfSeparator = filenames.at(i).indexOf(classSeparator);
        QString classString = filenames.at(i).left(indexOfSeparator);
        int subjectID = classString.toInt();

        Template t;
        t.subjectID = subjectID;
        t.featureVector = featureVector;

        templates.append(t);
    }

    return templates;
}

void Template::normalizeFeatureVectorComponents(QVector<Template> &templates,
                                                QVector<double> &minValues, QVector<double> &maxValues)
{
    for (int i = 0; i < templates.count(); i++)
    {
        templates[i].featureVector = Vector::normalizeComponents(templates[i].featureVector,
                                                                 minValues, maxValues);
    }
}

void Template::splitVectorsAndClasses(QVector<Template> &templates,
                                      QVector<Matrix> &featureVectors,
                                      QVector<int> &classMemberships)
{
    featureVectors.clear();
    classMemberships.clear();
    int n = templates.count();
    for (int i = 0; i < n; i++)
    {
        featureVectors.append(templates[i].featureVector);
        classMemberships.append(templates[i].subjectID);
    }
}

QVector<Template> Template::joinVectorsAndClasses(QVector<Matrix> &featureVectors,
                                                  QVector<int> &classMemberships)
{
    int n = featureVectors.count();
    assert(n == classMemberships.count());

    QVector<Template> templates;
    for (int i = 0; i < n; i++)
    {
        Template t;
        t.subjectID = classMemberships[i];
        t.featureVector = featureVectors[i];
        templates.append(t);
    }
    return templates;
}

void Template::saveTemplates(QList<QVector<Template> > &clusters, const QString &path, const QString &classSeparator)
{
    QMap<int, int> classUsage;
    for (int i = 0; i < clusters.count(); i++)
    {
        QVector<Template> &templates = clusters[i];
        for (int j = 0; j < templates.count(); j++)
        {
            Template &t = templates[j];

            if (!classUsage.contains(t.subjectID))
                classUsage[t.subjectID] = 0;
            classUsage[t.subjectID] += 1;

            QString filePath = path + QDir::separator() +
                    QString::number(t.subjectID) + classSeparator + QString::number(classUsage[t.subjectID]);

            Vector::toFile(t.featureVector, filePath);
        }
    }
}

ZScoreNormalizationResult Template::zScoreNorm(QVector<Template> &templates)
{
    QVector<Matrix> vectors = getVectors(templates);
    ZScoreNormalizationResult result = Normalization::zScoreNormalization(vectors);
    //zScoreNorm(templates, result);
    return result;
}

void Template::zScoreNorm(QVector<Template> &templates, ZScoreNormalizationResult &normParams)
{
    for (int templateIndex = 0; templateIndex < templates.count(); templateIndex++)
        Normalization::zScoreNormalization(templates[templateIndex].featureVector, normParams);
}

QVector<Matrix> Template::getVectors(QVector<Template> &templates)
{
    QVector<Matrix> result;
    for (int i = 0; i < templates.count(); i++)
    {
        result.append(templates[i].featureVector);
    }
    return result;
}

void Template::stats(QVector<Template> &templates, const QString &outPath)
{
    int n = templates.count();
    assert(n > 0);
    int m = templates[0].featureVector.rows;

    for (int featureIndex = 0; featureIndex < m; featureIndex++)
    {
        QVector<double> values;
        for (int templateIndex = 0; templateIndex < n; templateIndex++)
        {
            values.append(templates[templateIndex].featureVector(featureIndex));
        }

        double min = Vector::minValue(values);
        double max = Vector::maxValue(values);

        QVector<double> histogram(10);
        for (int i = 0; i < n; i++)
        {
            int bin = (int)((values[i]-min)/(max-min) * 10);
            if (bin == 10) bin = 9;
            histogram[bin] += 1;
        }
        for (int i = 0; i < 10; i++)
        {
            histogram[i] = histogram[i]/n;
        }

        QFile f(outPath + "-" + QString::number(featureIndex));
        f.open(QIODevice::WriteOnly);
        QTextStream out(&f);

        double x = min;
        double step = (max-min)/10;
        for (int i = 0; i < 10; i++)
        {
            out << x << ' ' << histogram[i] << '\n';
            x += step;
        }

        out.flush();
        f.close();
    }
}

QVector<Template> Template::createTemplates(QVector<Matrix> &rawData, QVector<int> &IDs, FeatureExtractor &extractor)
{
    int n = rawData.count();
    assert (n == IDs.count());

    QVector<Template> result;
    for (int i = 0; i < n; i++)
    {
        Template t;
        t.subjectID = IDs[i];
        t.featureVector = extractor.extract(rawData[i]);

        result.append(t);
    }

    return result;
}

QVector<Template> Template::clone(QVector<Template> &src)
{
	QVector<Template> result;
	for (int i = 0; i < src.count(); i++)
	{
		Template t;
		t.subjectID = src[i].subjectID;
		t.featureVector = src[i].featureVector.clone();
		result << t;
	}
	return result;
}
