#ifndef FACETEMPLATE_H
#define FACETEMPLATE_H

#include <QVector>

#include "linalg/vector.h"
#include "biometrics/zpcacorrw.h"
#include "biometrics/scorelevefusion.h"
#include "facelib/mesh.h"

class FilterBankClassifier; // forward declarations
class FilterBanksClassifiers;

class FilterBanksVectors
{
public:
    QVector<Vector> depth;
    QVector<Vector> index;
    QVector<Vector> mean;
    QVector<Vector> gauss;
    QVector<Vector> eigencur;
    QVector<Vector> textureE;

    void load(const QString &dirPath, const QString &baseFilename, const FilterBanksClassifiers &classifier);

    void load(const Matrix& image, QVector<Vector> &target, const FilterBankClassifier &classifier);

private:
    void load(const QString &dirPath, const QString &baseFilename, const QString &source, QVector<Vector> &target, const FilterBankClassifier &classifier);
};

class FaceClassifier; // forward declaration

class FaceTemplate
{
public:
    int id;

    Vector isocurves;
    FilterBanksVectors gaussLaguerreVectors;
    FilterBanksVectors gaborVectors;

    FaceTemplate() {}
    FaceTemplate(const QString &dirPath, const QString &baseFilename, const FaceClassifier &classifier);
    FaceTemplate(int id, const Mesh &properlyAlignedMesh, const FaceClassifier &classifier);
};

class FilterBankClassifier
{
public:
    QVector<Matrix> realWavelets;
    QVector<Matrix> imagWavelets;
    QVector<ZPCACorrW> projections;
    ScoreWeightedSumFusion fusion;

    FilterBankClassifier(const QString &source, bool isGabor);

    void load(const QString &dirPath, const QString &prefix1, const QString &prefix2);

    double compare(const QVector<Vector> &first, const QVector<Vector> &second);

    static void addFilterKernels(QVector<Matrix> &realWavelets, QVector<Matrix> &imagWavelets, const QString &source, bool gabor);
};

class FilterBanksClassifiers
{
public:
    FilterBankClassifier depth;
    FilterBankClassifier index;
    FilterBankClassifier mean;
    FilterBankClassifier gauss;
    FilterBankClassifier eigencur;
    FilterBankClassifier textureE;

    FilterBanksClassifiers(bool isGabor) :
        depth("depth", isGabor),
        index("index", isGabor),
        mean("mean", isGabor),
        gauss("gauss", isGabor),
        eigencur("eigencur", isGabor),
        textureE("textureE", isGabor) {}

    void load(const QString &dirPath, const QString &prefix);

    QVector<double> compare(const FilterBanksVectors &first, const FilterBanksVectors &second);
};

class FaceClassifier
{
public:
    ScoreSVMFusion fusion;
    ZPCACorrW isocurves;
    FilterBanksClassifiers gaussLaguerre;
    FilterBanksClassifiers gabor;

    FaceClassifier() : gaussLaguerre(false), gabor(true) {}
    FaceClassifier(const QString &dirPath);

    double compare(FaceTemplate &first, FaceTemplate &second);
};

#endif // FACETEMPLATE_H
