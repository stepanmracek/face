#ifndef TEMPLATE_H
#define TEMPLATE_H

#include <QVector>
#include <QList>
#include <QString>

#include "linalg/common.h"
#include "featureextractor.h"

class Template;
typedef QVector<Template> Templates;

class Template
{
public:
    int subjectID;
    Vector featureVector;

    Template() {}
    Template(int id, const Vector &featureVector) : subjectID(id), featureVector(featureVector) { }

    static QVector<Template> loadTemplates(const QString &dirPath, const QString& classSeparator);

    static void normalizeFeatureVectorComponents(QVector<Template> &templates,
                                                 QVector<double> &minValues, QVector<double> &maxValues);

    static void splitVectorsAndClasses(QVector<Template> &templates,
                                       QVector<Vector> &featureVectors,
                                       QVector<int> &classMemberships);

    static QVector<Template> joinVectorsAndClasses(QVector<Vector> &featureVectors,
                                                   QVector<int> &classMemberships);

    static void saveTemplates(QList<QVector<Template> > &clusters, const QString &path,
                              const QString &classSeparator = "-");

    static ZScoreNormalizationResult zScoreNorm(QVector<Template> &templates);

    static void zScoreNorm(QVector<Template> &templates, ZScoreNormalizationResult &normParams);

    static QVector<Vector> getVectors(QVector<Template> &templates);

    static void stats(QVector<Template> &templates, const QString &outPath);

    static QVector<Template> createTemplates(const QVector<Vector> &rawData,
                                             const QVector<int> &IDs,
                                             const FeatureExtractor &extractor);

    static QVector<Template> clone(const QVector<Template> &src);
};

#endif // TEMPLATE_H
