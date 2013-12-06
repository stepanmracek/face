#ifndef FACETEMPLATE_H
#define FACETEMPLATE_H

#include <QVector>
#include <QStringList>
#include <QMap>

#include "linalg/vector.h"
#include "biometrics/zpcacorrw.h"
#include "biometrics/scorelevefusion.h"
#include "facelib/mesh.h"

class FilterBankClassifier; // forward declarations
class FilterBanksClassifiers;

class FilterBanksVectors
{
public:
    QMap<QString, QVector<Vector> > source;

    //QVector<Vector> load(const QString &dirPath, const QString &baseFilename, const FilterBanksClassifiers &classifier);

    QVector<Vector> load(const Matrix& image, const FilterBankClassifier &classifier, double scaleFactor);

private:
    //QVector<Vector> load(const QString &dirPath, const QString &baseFilename, const QString &source, const FilterBankClassifier &classifier);
};

class FaceClassifier; // forward declaration

class Face3DTemplate
{
public:
    int id;

    Vector isocurves;
    QMap<QString, FilterBanksVectors> type;

    Face3DTemplate();
    //FaceTemplate(const QString &dirPath, const QString &baseFilename, const FaceClassifier &classifier);
    Face3DTemplate(int id, const Mesh &properlyAlignedMesh, const FaceClassifier &classifier);
    Face3DTemplate(int id, const QString &path, const FaceClassifier &classifier);
    Face3DTemplate(int id, cv::FileStorage & storage, const FaceClassifier &classifier);

    void serialize(const QString &path, const FaceClassifier &classifier) const;
    void serialize(cv::FileStorage & storage, const FaceClassifier &classifier) const;

    static Matrix getTexture(const Mesh &mesh);
    static QList<Matrix> getDeMeGaInEi(const Mesh &mesh);
    static QVector<VectorOfPoints> getIsoGeodesicCurves(const Mesh &mesh);

private:
    void deserialize(cv::FileStorage & storage, const FaceClassifier &classifier);
};

class FilterBankClassifier
{
public:
    QVector<Matrix> realWavelets;
    QVector<Matrix> imagWavelets;
    QVector<ZPCACorrW> projections;
    ScoreWeightedSumFusion fusion;

    FilterBankClassifier() {}
    FilterBankClassifier(const QString &source, bool isGabor);

    void load(const QString &dirPath, const QString &prefix1, const QString &prefix2);

    QVector<double> getScores();
    double compare(const QVector<Vector> &first, const QVector<Vector> &second) const;

    static void addFilterKernels(QVector<Matrix> &realWavelets, QVector<Matrix> &imagWavelets, const QString &source, bool gabor);
};

class FilterBanksClassifiers
{
public:
    QMap<QString, FilterBankClassifier> dict;

    FilterBanksClassifiers() {}
    FilterBanksClassifiers(bool isGabor);

    void load(const QString &dirPath, const QString &prefix);

    //QVector<double> compare(const FilterBanksVectors &first, const FilterBanksVectors &second);
};

class FaceClassifier
{
public:

    enum ComparisonType { CompareMinDistance, CompareMeanDistance, CompareMeanTemplate };

    ScoreSVMFusion fusion;
    ZPCACorrW isocurves;
    QMap<QString, FilterBanksClassifiers> bankClassifiers;
    QStringList units;

    FaceClassifier();
    FaceClassifier(const QString &dirPath);

    double compare(const Face3DTemplate *first, const Face3DTemplate *second, bool debug = false) const;
    double compare(const QList<Face3DTemplate *> &references, const Face3DTemplate *probe,
                   ComparisonType comparisonType, bool debug = false) const;

    Evaluation evaluate(const QVector<Face3DTemplate *> &templates) const;
    Evaluation evaluate(const QHash<int, Face3DTemplate *> &references, const QVector<Face3DTemplate *> &testTemplates,
                        ComparisonType comparisonType) const;

    QMap<int, double> identify(const QHash<int, Face3DTemplate *> &references, const Face3DTemplate *probe,
                               ComparisonType comparisonType) const;

    ScoreSVMFusion relearnFinalFusion(const QVector<Face3DTemplate *> &templates);
    void serialize(const QString &dirPath);
};

#endif // FACETEMPLATE_H
