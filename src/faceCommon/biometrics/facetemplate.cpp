#include "facetemplate.h"

#include <QDir>
#include "isocurveprocessing.h"
#include "linalg/serialization.h"
#include "linalg/matrixconverter.h"
#include "linalg/gabor.h"
#include "linalg/gausslaguerre.h"
#include "facelib/surfaceprocessor.h"
#include "biometrics/scorelevelfusionwrapper.h"

FilterBankClassifier::FilterBankClassifier(const QString &source, bool isGabor)
{
    addFilterKernels(realWavelets, imagWavelets, source, isGabor);
}

void FilterBankClassifier::load(const QString &dirPath, const QString &prefix1, const QString &prefix2)
{
    QDir pcaDir(dirPath, prefix1 + "-" + prefix2 + "-*-pca");
    QStringList pcaFiles = pcaDir.entryList();

    if (pcaFiles.count() == 0) return;

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

double FilterBankClassifier::compare(const QVector<Vector> &first, const QVector<Vector> &second) const
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

FilterBanksClassifiers::FilterBanksClassifiers(bool isGabor)
{
    dict["depth"] = FilterBankClassifier("depth", isGabor);
    dict["index"] = FilterBankClassifier("index", isGabor);
    dict["mean"] = FilterBankClassifier("mean", isGabor);
    dict["gauss"] = FilterBankClassifier("gauss", isGabor);
    dict["eigencur"] = FilterBankClassifier("eigencur", isGabor);
    dict["textureE"] = FilterBankClassifier("textureE", isGabor);
}

void FilterBanksClassifiers::load(const QString &dirPath, const QString &prefix)
{
    dict["depth"].load(dirPath, prefix, "depth");
    dict["index"].load(dirPath, prefix, "index");
    dict["gauss"].load(dirPath, prefix, "gauss");
    dict["mean"].load(dirPath, prefix, "mean");
    dict["eigencur"].load(dirPath, prefix, "eigencur");
    dict["textureE"].load(dirPath, prefix, "textureE");
}

FaceClassifier::FaceClassifier()
{
    bankClassifiers["gabor"] = FilterBanksClassifiers(true);
    bankClassifiers["gl"] = FilterBanksClassifiers(false);
}

FaceClassifier::FaceClassifier(const QString &dirPath) : fusion(dirPath + QDir::separator() + "final")
{
    // load units
    QString unitsPath = dirPath + QDir::separator() + "units";
    assert(QFile::exists(unitsPath));
    QFile unitsFile(unitsPath);
    unitsFile.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream unitsStream(&unitsFile);
    while (!unitsStream.atEnd())
    {
        units << unitsStream.readLine();
    }

    bankClassifiers["gabor"] = FilterBanksClassifiers(true);
    bankClassifiers["gl"] = FilterBanksClassifiers(false);

    QString isocurvesPca = dirPath + QDir::separator() + "isocurves-pca";
    QString isocurvesNormparams = dirPath + QDir::separator() + "isocurves-normparams";
    QString isocurvesWeights = dirPath + QDir::separator() + "isocurves-selWeights";
    if (QFile::exists(isocurvesPca) && QFile::exists(isocurvesNormparams) && QFile::exists(isocurvesWeights))
    {
        isocurves.extractor = ZScorePCAExtractor(isocurvesPca, isocurvesNormparams);
        isocurves.metric.w = Vector::fromFile(isocurvesWeights);
    }

    bankClassifiers["gl"].load(dirPath + QDir::separator(), "gl");
    bankClassifiers["gabor"].load(dirPath + QDir::separator(), "gabor");
}

double FaceClassifier::compare(const FaceTemplate *first, const FaceTemplate *second, bool debug) const
{
    QVector<double> scores;

    foreach (const QString &unitName, units)
    {
        if (unitName.compare("isocurves") == 0)
        {
            scores << isocurves.metric.distance(first->isocurves, second->isocurves);
        }
        else if (unitName.startsWith("gabor-") || unitName.startsWith("gl-"))
        {
            QStringList items = unitName.split("-");
            QString bankName = items[0];
            QString sourceName = items[1];
            scores << this->bankClassifiers[bankName].dict[sourceName].compare(
                        first->type[bankName].source[sourceName],
                        second->type[bankName].source[sourceName]);
        }
    }

    if (debug) qDebug() << scores;
    double d = this->fusion.fuse(scores);
    return d;
}

double FaceClassifier::compare(const QList<FaceTemplate *> &references, const FaceTemplate *probe, bool debug) const
{
    double n = references.count();
    double s = 0;
    foreach (const FaceTemplate *reference, references)
    {
        double score = compare(reference, probe, debug);
        s += score;

        //qDebug() << "  " << score;
    }

    return s/n;
}

Evaluation FaceClassifier::evaluate(const QVector<FaceTemplate*> &templates) const
{
    QHash<QPair<int, int>, double> distances;
    int n = templates.count();
    for (int i = 0; i < n - 1; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            double d = compare(templates[i], templates[j]);
            distances.insertMulti(QPair<int, int>(templates[i]->id, templates[j]->id), d);

            //qDebug() << templates[i].id << templates[j].id << (templates[i].id == templates[j].id) << d;
        }
    }

    return Evaluation(distances);
}

Evaluation FaceClassifier::evaluate(const QHash<int, FaceTemplate *> &references, const QVector<FaceTemplate *> &testTemplates) const
{
    QHash<QPair<int, int>, double> distances;

    foreach (const FaceTemplate *probe, testTemplates)
    {
        int probeID = probe->id;

        foreach (int referenceID, references.uniqueKeys())
        {
            double d = compare(references.values(referenceID), probe);
            distances.insertMulti(QPair<int, int>(referenceID, probeID), d);

            //qDebug() << referenceID << probeID << (referenceID == probeID) << d;
        }
    }

    return Evaluation(distances);
}

QMap<int, double> FaceClassifier::identify(const QHash<int, FaceTemplate *> &references, const FaceTemplate *probe) const
{
    QMap<int, double> result;

    foreach (int referenceID, references.uniqueKeys())
    {
        double d = compare(references.values(referenceID), probe);
        result[referenceID] = d;

        //qDebug() << referenceID << d;
    }

    return result;
}

ScoreSVMFusion FaceClassifier::relearnFinalFusion(const QVector<FaceTemplate> &templates)
{
    QMap<QString, QHash<QPair<int, int>, double> > distancesDict;
    int n = templates.count();
    foreach (const QString &unit, units)
    {
        distancesDict[unit] = QHash<QPair<int, int>, double>();
    }

    for (int i = 0; i < n - 1; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            const FaceTemplate &t1 = templates[i];
            const FaceTemplate &t2 = templates[j];
            QPair<int, int> pair(t1.id, t2.id);

            foreach (const QString &unitName, units)
            {
                double d;
                if (unitName.compare("isocurves") == 0)
                {
                    d = isocurves.metric.distance(t1.isocurves, t2.isocurves);
                }
                else if (unitName.startsWith("gabor-") || unitName.startsWith("gl-"))
                {
                    QStringList items = unitName.split("-");
                    QString bankName = items[0];
                    QString sourceName = items[1];
                    d = bankClassifiers[bankName].dict[sourceName].compare(
                                t1.type[bankName].source[sourceName],
                                t2.type[bankName].source[sourceName]);
                }

                distancesDict[unitName].insertMulti(pair, d);
            }
        }
    }


    QList<Evaluation> evaluations;
    foreach (const QString &unit, units)
    {
        Evaluation e(distancesDict[unit]);
        qDebug() << unit << e.eer;
        evaluations << e;
    }

    ScoreSVMFusion newFusion;
    ScoreLevelFusionWrapper wrapper;
    QVector<int> selectedIndicies = wrapper.trainClassifier(newFusion, evaluations);
    QStringList newUnits;
    foreach (int i, selectedIndicies)
    {
        newUnits << units[i];
    }
    units = newUnits;
    return newFusion;
}

void FaceClassifier::serialize(const QString &dirPath)
{
    // units
    QString unitsPath = dirPath + QDir::separator() + "units";
    QFile unitsFile(unitsPath);
    unitsFile.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream unitsStream(&unitsFile);

    // final fusion
    fusion.serialize(dirPath + QDir::separator() + "final");

    // individual units
    foreach (const QString &unit, units)
    {
        // unit name
        unitsStream << unit << "\n";

        if (unit.compare("isocurves") == 0)
        {
            QString isocurvesPca = dirPath + QDir::separator() + "isocurves-pca";
            QString isocurvesNormparams = dirPath + QDir::separator() + "isocurves-normparams";
            QString isocurvesWeights = dirPath + QDir::separator() + "isocurves-selWeights";
            isocurves.extractor.serialize(isocurvesPca, isocurvesNormparams);
            isocurves.metric.w.toFile(isocurvesWeights);
        }
        else if (unit.startsWith("gabor-") || unit.startsWith("gl-"))
        {
            QStringList items = unit.split("-");
            QString bankName = items[0];
            QString sourceName = items[1];

            const FilterBankClassifier &c = bankClassifiers[bankName].dict[sourceName];
            c.fusion.serialize(dirPath + QDir::separator() + unit + "-wSumFusion");
            int count = c.projections.count();
            for (int i = 0; i < count; i++)
            {
                QString pcaPath = dirPath + QDir::separator() + unit + "-" + QString::number(i) + "-pca";
                QString normParamsPath = dirPath + QDir::separator() + unit + "-" + QString::number(i) + "-normParams";
                c.projections[i].extractor.serialize(pcaPath, normParamsPath);
            }
        }
    }
}

FaceTemplate::FaceTemplate()
{
    type["gl"] = FilterBanksVectors();
    type["gabor"] = FilterBanksVectors();
}

FaceTemplate::FaceTemplate(const QString &dirPath, const QString &baseFilename, const FaceClassifier &classifier)
{
    type["gl"] = FilterBanksVectors();
    type["gabor"] = FilterBanksVectors();
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
    type["gl"].load(path, baseFilename, classifier.bankClassifiers["gl"]);

    // gabor
    type["gabor"].load(path, baseFilename, classifier.bankClassifiers["gabor"]);
}

FaceTemplate::FaceTemplate(int id, const Mesh &properlyAlignedMesh, const FaceClassifier &classifier)
{
    type["gl"] = FilterBanksVectors();
    type["gabor"] = FilterBanksVectors();
    this->id = id;

    Matrix smoothKernel5 = KernelGenerator::gaussianKernel(5);
    Matrix smoothKernel7 = KernelGenerator::gaussianKernel(7);
    cv::Rect roi(25, 15, 100, 90);
    MapConverter converter;

    // depth
    Map depthmap = SurfaceProcessor::depthmap(properlyAlignedMesh, converter,
                                              cv::Point2d(-75, -75), cv::Point2d(75, 75),
                                              1, ZCoord);
    depthmap.bandPass(-75, 0, false, false);
    depthmap.applyFilter(smoothKernel5, 2, true);
    Matrix depthImage = depthmap.toMatrix(0, -75, 0);
    depthImage = depthImage(roi);

    if (classifier.units.contains("gabor-depth"))
        type["gabor"].source["depth"] = type["gabor"].load(depthImage, classifier.bankClassifiers["gabor"].dict["depth"]);
    if (classifier.units.contains("gl-depth"))
        type["gl"].source["depth"] = type["gl"].load(depthImage, classifier.bankClassifiers["gl"].dict["depth"]);

    depthmap.applyFilter(smoothKernel7, 3, true);
    CurvatureStruct cs = SurfaceProcessor::calculateCurvatures(depthmap);

    // mean
    cs.curvatureMean.bandPass(-0.1, 0.1, false, false);
    Matrix meanImage = cs.curvatureMean.toMatrix(0, -0.1, 0.1);
    meanImage = meanImage(roi);

    if (classifier.units.contains("gabor-mean"))
        type["gabor"].source["mean"] = type["gabor"].load(meanImage, classifier.bankClassifiers["gabor"].dict["mean"]);

    if (classifier.units.contains("gl-mean"))
        type["gl"].source["mean"] = type["gl"].load(meanImage, classifier.bankClassifiers["gl"].dict["mean"]);


    // gauss
    cs.curvatureGauss.bandPass(-0.01, 0.01, false, false);
    Matrix gaussImage = cs.curvatureGauss.toMatrix(0, -0.01, 0.01);
    gaussImage = gaussImage(roi);

    if (classifier.units.contains("gabor-gauss"))
        type["gabor"].source["gauss"] = type["gabor"].load(gaussImage, classifier.bankClassifiers["gabor"].dict["gauss"]);

    if (classifier.units.contains("gl-gauss"))
        type["gl"].source["gauss"] = type["gl"].load(gaussImage, classifier.bankClassifiers["gl"].dict["gauss"]);

    // index
    cs.curvatureIndex.bandPass(0, 1, false, false);
    Matrix indexImage = cs.curvatureIndex.toMatrix(0, 0, 1);
    indexImage = indexImage(roi);

    if (classifier.units.contains("gabor-index"))
        type["gabor"].source["index"] = type["gabor"].load(indexImage, classifier.bankClassifiers["gabor"].dict["index"]);

    if (classifier.units.contains("gl-index"))
        type["gl"].source["index"] = type["gl"].load(indexImage, classifier.bankClassifiers["gl"].dict["index"]);

    // eigencur
    cs.curvaturePcl.bandPass(0, 0.0025, false, false);
    Matrix eigencurImage = cs.curvaturePcl.toMatrix(0, 0, 0.0025);
    eigencurImage = eigencurImage(roi);

    if (classifier.units.contains("gabor-eigencur"))
        type["gabor"].source["eigencur"] = type["gabor"].load(eigencurImage, classifier.bankClassifiers["gabor"].dict["eigencur"]);

    if (classifier.units.contains("gl-eigencur"))
        type["gl"].source["eigencur"] = type["gl"].load(eigencurImage, classifier.bankClassifiers["gl"].dict["eigencur"]);

    // texture
    Map textureMap = SurfaceProcessor::depthmap(properlyAlignedMesh, converter,
                                                cv::Point2d(-75, -75), cv::Point2d(75, 75),
                                                1, Texture_I);
    Matrix textureImage = textureMap.toMatrix(0, 0, 255);
    ImageGrayscale textureGSImage = MatrixConverter::DoubleMatrixToGrayscaleImage(textureImage);
    cv::equalizeHist(textureGSImage, textureGSImage);
    textureImage = MatrixConverter::grayscaleImageToDoubleMatrix(textureGSImage);
    textureImage = textureImage(roi);

    if (classifier.units.contains("gabor-textureE"))
        type["gabor"].source["textureE"] = type["gabor"].load(textureImage, classifier.bankClassifiers["gabor"].dict["textureE"]);

    if (classifier.units.contains("gl-textureE"))
        type["gl"].source["textureE"] = type["gl"].load(textureImage, classifier.bankClassifiers["gl"].dict["textureE"]);

    // isocurves
    if (classifier.units.contains("isocurves"))
    {
        cv::Point3d center(0,20,0);
        VectorOfCurves isocurves;
        for (int distance = 10; distance <= 50; distance += 10)
        {
            VectorOfPoints isocurve = SurfaceProcessor::isoGeodeticCurve(depthmap, converter, center, distance, 100, 1);
            isocurves << isocurve;
        }

        this->isocurves = classifier.isocurves.extractor.extract(IsoCurveProcessing::generateFeatureVector(isocurves, false));
    }
}

FaceTemplate::FaceTemplate(int id, const QString &path, const FaceClassifier &classifier)
{
    this->id = id;
    cv::FileStorage storage(path.toStdString(), cv::FileStorage::READ);

    foreach(const QString &unit, classifier.units)
    {
        if (unit.compare("isocurves") == 0)
        {
            storage["isocurves"] >> isocurves;
        }
        else if (unit.startsWith("gabor-") || unit.startsWith("gl-"))
        {
            QStringList items = unit.split("-");
            QString bankName = items[0];
            QString sourceName = items[1];

            if (!type.contains(bankName))
                type[bankName] = FilterBanksVectors();

            cv::FileNode node = storage[unit.toStdString()];
            for (cv::FileNodeIterator it = node.begin(); it != node.end(); ++it)
            {
                Matrix m;
                (*it) >> m;

                if (!type[bankName].source.contains(sourceName))
                    type[bankName].source[sourceName] = QVector<Vector>();

                type[bankName].source[sourceName] << Vector(m);
            }

            /*storage << unit.toStdString() << "[";

            const QVector<Vector> &vectors = type[bankName].source[sourceName];
            foreach(const Vector &v, vectors)
            {
                storage << v;
            }

            storage << "]";*/
        }
    }

}

void FaceTemplate::serialize(const QString &path, const FaceClassifier &classifier) const
{
    cv::FileStorage storage(path.toStdString(), cv::FileStorage::WRITE);

    foreach(const QString &unit, classifier.units)
    {
        if (unit.compare("isocurves") == 0)
        {
            storage << unit.toStdString() << isocurves;
        }
        else if (unit.startsWith("gabor-") || unit.startsWith("gl-"))
        {
            QStringList items = unit.split("-");
            QString bankName = items[0];
            QString sourceName = items[1];
            storage << unit.toStdString() << "[";

            const QVector<Vector> &vectors = type[bankName].source[sourceName];
            foreach(const Vector &v, vectors)
            {
                storage << v;
            }

            storage << "]";
        }
    }
}

QVector<Vector> FilterBanksVectors::load(const QString &dirPath, const QString &baseFilename, const FilterBanksClassifiers &classifier)
{
    QStringList srcNames;
    srcNames << "depth" << "index" << "gauss" << "mean" << "eigencur" << "textureE";
    foreach (const QString &srcName, srcNames)
    {
        source[srcName] = load(dirPath, baseFilename, srcName, classifier.dict[srcName]);
    }
}

QVector<Vector> FilterBanksVectors::load(const Matrix &image, const FilterBankClassifier &classifier)
{
    QVector<Vector> result;
    for (int i = 0; i < classifier.realWavelets.count(); i++)
    {
        Vector rawVector = (classifier.realWavelets[i].rows == 0) ?
                    MatrixConverter::matrixToColumnVector(image) :
                    MatrixConverter::matrixToColumnVector(FilterBank::absResponse(image, classifier.realWavelets[i], classifier.imagWavelets[i]));
        result << classifier.projections[i].extractor.extract(rawVector);
    }
    return result;
}

QVector<Vector> FilterBanksVectors::load(const QString &dirPath, const QString &baseFilename, const QString &source, const FilterBankClassifier &classifier)
{
    Matrix img = MatrixConverter::imageToMatrix(dirPath + source + QDir::separator() + baseFilename + ".png");
    cv::Rect roi(25, 15, 100, 90);
    img = img(roi);

    return load(img, classifier);
}
