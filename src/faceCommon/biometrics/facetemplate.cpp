#include "facetemplate.h"

#include <QDir>
#include "isocurveprocessing.h"
#include "linalg/serialization.h"
#include "linalg/matrixconverter.h"
#include "linalg/gabor.h"
#include "linalg/gausslaguerre.h"

FilterBankClassifier::FilterBankClassifier(const QString &source, bool isGabor)
{
    addFilterKernels(realWavelets, imagWavelets, source, isGabor);
}

void FilterBankClassifier::load(const QString &dirPath, const QString &prefix1, const QString &prefix2)
{
    QDir pcaDir(dirPath, prefix1 + "-" + prefix2 + "-*-pca");
    QStringList pcaFiles = pcaDir.entryList();

    QDir normParamsDir(dirPath, prefix1 + "-" + prefix2 + "-*-normParams");
    QStringList normParamsFiles = normParamsDir.entryList();

    int n = pcaFiles.count();
    assert(n == normParamsFiles.count());
    this->projections.resize(n);

    for (int i = 0; i < n; i++)
    {
        int index = pcaFiles[i].split("-")[2].toInt();
        qDebug() << prefix1.toStdString().c_str() << prefix2.toStdString().c_str() << i;
        projections[index].extractor = ZScorePCAExtractor(dirPath + pcaFiles[i], dirPath + normParamsFiles[i]);
        projections[index].metric.w = Vector(projections[index].extractor.pca.getModes(), 1.0);
    }

    fusion = ScoreWeightedSumFusion(dirPath + prefix1 + "-" + prefix2 + "-wSumFusion");
}

double FilterBankClassifier::compare(const QVector<Vector> &first, const QVector<Vector> &second)
{
    int n = first.count();
    assert(n == second.count());
    assert(n == projections.count());
    QVector<double> scores;
    for (int i = 0; i < n; i++)
    {
        scores << projections[i].metric.distance(first[i], second[i]);
    }
    double result = fusion.fuse(scores);
    return result;
}

void FilterBanksClassifiers::load(const QString &dirPath, const QString &prefix)
{
    depth.load(dirPath, prefix, "depth");
    index.load(dirPath, prefix, "index");
    gauss.load(dirPath, prefix, "gauss");
    mean.load(dirPath, prefix, "mean");
    eigencur.load(dirPath, prefix, "eigencur");
    textureE.load(dirPath, prefix, "textureE");
}

QVector<double> FilterBanksClassifiers::compare(const FilterBanksVectors &first, const FilterBanksVectors &second)
{
    QVector<double> result;
    result << index.compare(first.index, second.index)
           << mean.compare(first.mean, second.mean)
           << gauss.compare(first.gauss, second.gauss)
           << eigencur.compare(first.eigencur, second.eigencur)
           << depth.compare(first.depth, second.depth)
           << textureE.compare(first.textureE, second.textureE);
    return result;
}

FaceClassifier::FaceClassifier(const QString &dirPath) : fusion(dirPath + QDir::separator() + "final"),  gaussLaguerre(false), gabor(true)
{
    isocurves.extractor = ZScorePCAExtractor(dirPath + QDir::separator() + "isocurves-pca",
                                             dirPath + QDir::separator() + "isocurves-normparams");
    isocurves.metric.w = Vector::fromFile(dirPath + QDir::separator() + "isocurves-selWeights");

    gaussLaguerre.load(dirPath + QDir::separator(), "gl");
    gabor.load(dirPath + QDir::separator(), "gabor");
}

double FaceClassifier::compare(FaceTemplate &first, FaceTemplate &second)
{
    QVector<double> scores;
    scores << isocurves.metric.distance(first.isocurves, second.isocurves);
    scores += gaussLaguerre.compare(first.gaussLaguerreVectors, second.gaussLaguerreVectors);
    scores += gabor.compare(first.gaborVectors, second.gaborVectors);
    double result = fusion.fuse(scores);
    return result;
}

FaceTemplate::FaceTemplate(const QString &dirPath, const QString &baseFilename, const FaceClassifier &classifier)
{
    QString path = dirPath.endsWith(QDir::separator()) ? dirPath : dirPath + QDir::separator();

    id = baseFilename.split("d")[0].toInt();

    // isocurves
    VectorOfCurves curves = Serialization::readVectorOfPointclouds(path + "isocurves2" + QDir::separator() + baseFilename + ".xml");
    VectorOfCurves selected;
    for (int i = 0; i < 5; i++)
    {
        selected << curves[i];
    }
    isocurves = classifier.isocurves.extractor.extract(IsoCurveProcessing::generateFeatureVector(selected, false));

    // gauss-laguerre
    gaussLaguerreVectors.load(path, baseFilename, classifier.gaussLaguerre);

    // gabor
    gaborVectors.load(path, baseFilename, classifier.gabor);
}

void FilterBanksVectors::load(const QString &dirPath, const QString &baseFilename, const FilterBanksClassifiers &classifier)
{
    load(dirPath, baseFilename, "depth", depth, classifier.depth);
    load(dirPath, baseFilename, "index", index, classifier.index);
    load(dirPath, baseFilename, "gauss", gauss, classifier.gauss);
    load(dirPath, baseFilename, "mean", mean, classifier.mean);
    load(dirPath, baseFilename, "eigencur", eigencur, classifier.eigencur);
    load(dirPath, baseFilename, "textureE", textureE, classifier.textureE);
}

void FilterBanksVectors::load(const QString &dirPath, const QString &baseFilename, const QString &source, QVector<Vector> &target, const FilterBankClassifier &classifier)
{
    Matrix img = MatrixConverter::imageToMatrix(dirPath + source + QDir::separator() + baseFilename + ".png");
    cv::Rect roi(25, 15, 100, 90);
    img = img(roi);

    for (int i = 0; i < classifier.realWavelets.count(); i++)
    {
        Vector rawVector = (classifier.realWavelets[i].rows == 0) ?
                    MatrixConverter::matrixToColumnVector(img) :
                    MatrixConverter::matrixToColumnVector(FilterBank::absResponse(img, classifier.realWavelets[i], classifier.imagWavelets[i]));
        target << classifier.projections[i].extractor.extract(rawVector);
    }
}

void FilterBankClassifier::addFilterKernels(QVector<Matrix> &realWavelets, QVector<Matrix> &imagWavelets, const QString &source, bool gabor)
{
    int kSize = 200;

    QVector<int> p1; QVector<int> p2;

    // Index
    if (source.compare("index") == 0)
    {
        if (gabor)
        {
            p1 << 5 << 5 << 6 << 6 << 4 << 5 << 6 << 5 << 4 << 6 << 4;
            p2 << 8 << 2 << 4 << 1 << 3 << 5 << 3 << 4 << 8 << 2 << 1;
        }
        else
        {
            p1 << 75 << 0 << 75 << 75 << 100;
            p2 <<  2 << 0 <<  3 <<  5 <<   1;
        }
    }

    // Mean
    else if (source.compare("mean") == 0)
    {
        if (gabor)
        {
            p1 << 5 << 6 << 6 << 6 << 4 << 6 << 6 << 5;
            p2 << 8 << 3 << 8 << 4 << 2 << 6 << 7 << 1;
        }
        else
        {
            p1 << 75 << 100 << 50 << 100 << 25 << 75 << 75 << 0;
            p2 <<  3 <<   2 <<  2 <<   4 <<  5 <<  1 <<  5 << 0;
        }
    }

    // Depth
    else if (source.compare("depth") == 0)
    {
        if (gabor)
        {
            p1 << 6 << 6 << 6 << 5;
            p2 << 8 << 3 << 6 << 1;
        }
        else
        {
            p1 << 50 << 50 << 0 << 75 << 25 << 100;
            p2 <<  1 <<  2 << 0 <<  3 <<  1 <<   4;
        }
    }

    // Gauss
    else if (source.compare("gauss") == 0)
    {
        if (gabor)
        {
            p1 << 5 << 4 << 6 << 4 << 4 << 5 << 4;
            p2 << 7 << 4 << 3 << 1 << 5 << 2 << 2;
        }
        else
        {
            p1 << 50 << 75 << 100 << 75 << 25 << 50 << 100;
            p2 <<  1 <<  1 <<   5 <<  5 <<  5 <<  2 <<   4;
        }
    }

    // Eigencur
    else if (source.compare("eigencur") == 0)
    {
        if (gabor)
        {
            p1 << 4 << 5 << 5 << 4 << 6 << 4 << 5;
            p2 << 8 << 3 << 7 << 1 << 6 << 4 << 8;
        }
        else
        {
            p1 << 75 << 50 << 100 << 50 << 75 << 50 << 100 << 25 << 75 << 75;
            p2 <<  4 <<  1 <<   5 << 3 <<   2 <<  4 <<   1 <<  5 <<  3 <<  1;
        }
    }

    else if (source.compare("textureE") == 0)
    {
        if (gabor)
        {
            p1 << 5 << 6 << 5 << 6 << 6 << 4;
            p2 << 4 << 3 << 6 << 1 << 5 << 2;
        }
        else
        {
            p1 << 0 << 100 << 100 << 25 << 100 << 75;
            p2 << 0 <<   3 <<   1 <<  5 <<   4 <<  5;
        }
    }

    for (int i = 0; i < p1.count(); i++)
    {
        if (p1[i] == 0 && p2[i] == 0)
        {
            realWavelets << Matrix(0, 0);
            imagWavelets << Matrix(0, 0);
        }
        else
        {
            if (gabor)
            {
                realWavelets << Matrix(kSize, kSize);
                imagWavelets << Matrix(kSize, kSize);
                Gabor::createWavelet(realWavelets[i], imagWavelets[i], p1[i], p2[i]);
            }
            else
            {
                realWavelets << Matrix(p1[i], p1[i]);
                imagWavelets << Matrix(p1[i], p1[i]);
                GaussLaguerre::createWavelet(realWavelets[i], imagWavelets[i], p2[i], 0, 0);
            }
        }
    }
}
