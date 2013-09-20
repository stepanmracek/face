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

    QVector<Vector> load(const QString &dirPath, const QString &baseFilename, const FilterBanksClassifiers &classifier);

    QVector<Vector> load(const Matrix& image, const FilterBankClassifier &classifier);

private:
    QVector<Vector> load(const QString &dirPath, const QString &baseFilename, const QString &source, const FilterBankClassifier &classifier);
};

class FaceClassifier; // forward declaration

class FaceTemplate
{
public:
    int id;

    Vector isocurves;
    QMap<QString, FilterBanksVectors> type;

    FaceTemplate();
    FaceTemplate(const QString &dirPath, const QString &baseFilename, const FaceClassifier &classifier);
    FaceTemplate(int id, const Mesh &properlyAlignedMesh, const FaceClassifier &classifier);
    FaceTemplate(int id, const QString &path, const FaceClassifier &classifier);

    void serialize(const QString &path, const FaceClassifier &classifier);
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
    double compare(const QVector<Vector> &first, const QVector<Vector> &second);

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
    ScoreSVMFusion fusion;
    ZPCACorrW isocurves;
    QMap<QString, FilterBanksClassifiers> bankClassifiers;
    QStringList units;

    FaceClassifier();
    FaceClassifier(const QString &dirPath);

    double compare(const FaceTemplate &first, const FaceTemplate &second);
    Evaluation evaluate(const QVector<FaceTemplate> &templates);
    ScoreSVMFusion relearnFinalFusion(const QVector<FaceTemplate> &templates);
    void serialize(const QString &dirPath);
};

#endif // FACETEMPLATE_H
