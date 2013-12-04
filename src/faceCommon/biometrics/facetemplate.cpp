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
    QVector<int> p1; QVector<int> p2;

    // Index
    if (source.compare("index") == 0)
    {
        if (gabor)
        {
            p1 << 5 << 4 << 0 << 4 << 4 << 4;
            p2 << 2 << 8 << 0 << 3 << 6 << 1;
        }
        else
        {
            p1 << 0 << 75 << 50 << 100 << 100 << 75;
            p2 << 0 << 1 << 3 << 3 << 2 << 2;
        }
    }

    // Mean
    else if (source.compare("mean") == 0)
    {
        if (gabor)
        {
            p1 << 4 << 0 << 6 << 6 << 4 << 4 << 4 << 5 << 6;
            p2 << 8 << 0 << 3 << 6 << 1 << 3 << 4 << 3 << 8;
        }
        else
        {
            p1 << 75 << 50 << 0 << 100 << 75;
            p2 << 1 << 3 << 0 << 4 << 4;
        }
    }

    // Depth
    else if (source.compare("depth") == 0)
    {
        if (gabor)
        {
            p1 << 4 << 6 << 4 << 6;
            p2 << 8 << 4 << 1 << 3;
        }
        else
        {
            p1 << 50 << 50 << 75 << 50 << 25 << 75;
            p2 << 1 << 3 << 1 << 2 << 4 << 2;
        }
    }

    // Gauss
    else if (source.compare("gauss") == 0)
    {
        if (gabor)
        {
            p1 << 4 << 5 << 5 << 4 << 4 << 5 << 4 << 6 << 4;
            p2 << 8 << 6 << 2 << 1 << 6 << 8 << 4 << 1 << 7;
        }
        else
        {
            p1 << 25 << 100 << 50 << 100 << 75 << 50;
            p2 << 1 << 4 << 4 << 1 << 5 << 2;
        }
    }

    // Eigencur
    else if (source.compare("eigencur") == 0)
    {
        if (gabor)
        {
            p1 << 5 << 4 << 4 << 5 << 0 << 5 << 6 << 4;
            p2 << 7 << 2 << 4 << 3 << 0 << 8 << 6 << 1;
        }
        else
        {
            p1 << 50 << 50 << 100 << 75;
            p2 << 5 << 2 << 5 << 1;
        }
    }

    // Texture
    else if (source.compare("textureE") == 0)
    {
        if (gabor)
        {
            p1 << 0 << 5 << 5 << 6 << 6 << 6 << 5 << 5 << 4 << 4;
            p2 << 0 << 3 << 5 << 1 << 4 << 7 << 1 << 6 << 5 << 7;
        }
        else
        {
            p1 << 0 << 100 << 50 << 25 << 75 << 100 << 50;
            p2 << 0 << 3 << 4 << 5 << 1 << 4 << 3;
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
                realWavelets << Matrix();
                imagWavelets << Matrix();
                Gabor::createWavelet(realWavelets[i], imagWavelets[i], p1[i], p2[i]);
            }
            else
            {
                realWavelets << Matrix();
                imagWavelets << Matrix();
                GaussLaguerre::createWavelet(realWavelets[i], imagWavelets[i], p1[i], p2[i], 0);
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

double FaceClassifier::compare(const Face3DTemplate *first, const Face3DTemplate *second, bool debug) const
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

double FaceClassifier::compare(const QList<Face3DTemplate *> &references, const Face3DTemplate *probe,
                               ComparisonType comparisonType, bool debug) const
{
    double n = references.count();
    double s = 0;

    QVector<double> scores;
    foreach (const Face3DTemplate *reference, references)
    {
        double score = compare(reference, probe, debug);
        scores << score;
    }

    switch (comparisonType)
    {
    case FaceClassifier::CompareMinDistance:
        return Vector(scores).minValue();
    case FaceClassifier::CompareMeanDistance:
        return Vector(scores).meanValue();
    case FaceClassifier::CompareMeanTemplate:
    default:
        return 0;
    }

    return s/n;
}

Evaluation FaceClassifier::evaluate(const QVector<Face3DTemplate*> &templates) const
{
    QHash<QPair<int, int>, double> distances;
    int n = templates.count();
    for (int i = 0; i < n - 1; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            double d = compare(templates[i], templates[j]);
            distances.insertMulti(QPair<int, int>(templates[i]->id, templates[j]->id), d);

            //qDebug() << templates[i]->id << templates[j]->id << (templates[i]->id == templates[j]->id) << d;
        }
    }

    return Evaluation(distances);
}

Evaluation FaceClassifier::evaluate(const QHash<int, Face3DTemplate *> &references,
                                    const QVector<Face3DTemplate *> &testTemplates,
                                    ComparisonType comparisonType) const
{
    QHash<QPair<int, int>, double> distances;

    foreach (const Face3DTemplate *probe, testTemplates)
    {
        int probeID = probe->id;

        foreach (int referenceID, references.uniqueKeys())
        {
            double d = compare(references.values(referenceID), probe, comparisonType);
            distances.insertMulti(QPair<int, int>(referenceID, probeID), d);

            //qDebug() << referenceID << probeID << (referenceID == probeID) << d;
        }
    }

    return Evaluation(distances);
}

QMap<int, double> FaceClassifier::identify(const QHash<int, Face3DTemplate *> &references,
                                           const Face3DTemplate *probe,
                                           ComparisonType comparisonType) const
{
    QMap<int, double> result;

    foreach (int referenceID, references.uniqueKeys())
    {
        double d = compare(references.values(referenceID), probe, comparisonType);
        result[referenceID] = d;

        //qDebug() << referenceID << d;
    }

    return result;
}

ScoreSVMFusion FaceClassifier::relearnFinalFusion(const QVector<Face3DTemplate*> &templates)
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
            const Face3DTemplate *t1 = templates[i];
            const Face3DTemplate *t2 = templates[j];
            QPair<int, int> pair(t1->id, t2->id);

            foreach (const QString &unitName, units)
            {
                double d;
                if (unitName.compare("isocurves") == 0)
                {
                    d = isocurves.metric.distance(t1->isocurves, t2->isocurves);
                }
                else if (unitName.startsWith("gabor-") || unitName.startsWith("gl-"))
                {
                    QStringList items = unitName.split("-");
                    QString bankName = items[0];
                    QString sourceName = items[1];
                    d = bankClassifiers[bankName].dict[sourceName].compare(
                                t1->type[bankName].source[sourceName],
                                t2->type[bankName].source[sourceName]);
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

Face3DTemplate::Face3DTemplate()
{
    type["gl"] = FilterBanksVectors();
    type["gabor"] = FilterBanksVectors();
}

/*FaceTemplate::FaceTemplate(const QString &dirPath, const QString &baseFilename, const FaceClassifier &classifier)
{
    type["gl"] = FilterBanksVectors();
    type["gabor"] = FilterBanksVectors();
    QString path = dirPath.endsWith(QDir::separator()) ? dirPath : dirPath + QDir::separator();

    id = baseFilename.split("d")[0].toInt();

    // isocurves
    VectorOfCurves curves = Serialization::readVectorOfPointclouds(path + "isocurves" + QDir::separator() + baseFilename + ".xml");
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
}*/

Matrix Face3DTemplate::getTexture(const Mesh &mesh)
{
    cv::Rect roi(25, 15, 100, 90);
    MapConverter converter;
    Map texture = SurfaceProcessor::depthmap(mesh, converter, cv::Point2d(-75, -75), cv::Point2d(75, 75), 1, Texture_I);
    Matrix image = texture.toMatrix(0, 0, 255)(roi);
    Matrix equalized =  MatrixConverter::equalize(image);
    return equalized;
}

QList<Matrix> Face3DTemplate::getDeMeGaInEi(const Mesh &mesh)
{
    QList<Matrix> result;

    cv::Rect roi(25, 15, 100, 90);
    MapConverter converter;
    Map depthmap = SurfaceProcessor::depthmap(mesh, converter, cv::Point2d(-75, -75), cv::Point2d(75, 75), 1, ZCoord);

    //cv::imshow("getDeMeGaInEi1", depthmap.toMatrix());
    depthmap.bandPass(-70, 10, false, false);
    //cv::imshow("getDeMeGaInEi2", depthmap.toMatrix());

    result << depthmap.toMatrix(0, -70, 10)(roi);

    Matrix smoothKernel = KernelGenerator::gaussianKernel(5);

    Map smoothedDepthmap = depthmap;
    smoothedDepthmap.applyFilter(smoothKernel, 7, true);
    //for (int i = 0; i < 7; i++)
    //    cv::GaussianBlur(smoothedDepthmap.values, smoothedDepthmap.values, cv::Size(5, 5), -1);

    //cv::imshow("getDeMeGaInEi3", smoothedDepthmap.toMatrix());

    CurvatureStruct cs = SurfaceProcessor::calculateCurvatures(smoothedDepthmap);

    //cv::imshow("4", cs.curvatureMean.toMatrix());
    cs.curvatureMean.bandPass(-0.1, 0.1, false, false);
    result << cs.curvatureMean.toMatrix(0, -0.1, 0.1)(roi);



    cs.curvatureGauss.bandPass(-0.01, 0.01, false, false);
    result << cs.curvatureGauss.toMatrix(0, -0.01, 0.01)(roi);

    cs.curvatureIndex.bandPass(0, 1, false, false);
    result << cs.curvatureIndex.toMatrix(0, 0, 1)(roi);

    cs.curvaturePcl.bandPass(0, 0.0025, false, false);
    result << cs.curvaturePcl.toMatrix(0, 0, 0.0025)(roi);

    //cv::waitKey();
    return result;
}

VectorOfCurves Face3DTemplate::getIsoGeodesicCurves(const Mesh &mesh)
{
    MapConverter converter;
    Map depth = SurfaceProcessor::depthmap(mesh, converter, 2, ZCoord);
    Matrix gaussKernel = KernelGenerator::gaussianKernel(7);
    depth.applyFilter(gaussKernel, 3, true);
    //for (int i = 0; i < 3; i++)
    //    cv::GaussianBlur(depth.values, depth.values, cv::Size(7, 7), -1);


    QVector<VectorOfPoints> isoCurves;
    int startD = 10;
    for (int d = startD; d <= 50; d += 10)
    {
        VectorOfPoints isoCurve = SurfaceProcessor::isoGeodeticCurve(depth, converter, cv::Point3d(0,20,0), d, 100, 2);
        isoCurves << isoCurve;
    }

    return isoCurves;
}

Face3DTemplate::Face3DTemplate(int id, const Mesh &properlyAlignedMesh, const FaceClassifier &classifier)
{
    type["gl"] = FilterBanksVectors();
    type["gabor"] = FilterBanksVectors();
    this->id = id;

    double scaleFactor = 0.5;

    Matrix texture = getTexture(properlyAlignedMesh);
    QList<Matrix> curvatures = getDeMeGaInEi(properlyAlignedMesh);

    qDebug() << "depth";
    if (classifier.units.contains("gabor-depth"))
    {
        type["gabor"].source["depth"] = type["gabor"].load(
                    curvatures[0], classifier.bankClassifiers["gabor"].dict["depth"], scaleFactor);
    }
    if (classifier.units.contains("gl-depth"))
    {
        type["gl"].source["depth"] = type["gl"].load(
                    curvatures[0], classifier.bankClassifiers["gl"].dict["depth"], scaleFactor);
    }

    // mean
    qDebug() << "mean";
    if (classifier.units.contains("gabor-mean"))
    {
        type["gabor"].source["mean"] = type["gabor"].load(
                    curvatures[1], classifier.bankClassifiers["gabor"].dict["mean"], scaleFactor);
    }
    if (classifier.units.contains("gl-mean"))
    {
        type["gl"].source["mean"] = type["gl"].load(
                    curvatures[1], classifier.bankClassifiers["gl"].dict["mean"], scaleFactor);
    }

    // gauss
    qDebug() << "gauss";
    if (classifier.units.contains("gabor-gauss"))
    {
        type["gabor"].source["gauss"] = type["gabor"].load(
                    curvatures[2], classifier.bankClassifiers["gabor"].dict["gauss"], scaleFactor);
    }
    if (classifier.units.contains("gl-gauss"))
    {
        type["gl"].source["gauss"] = type["gl"].load(
                    curvatures[2], classifier.bankClassifiers["gl"].dict["gauss"], scaleFactor);
    }

    // index
    qDebug() << "index";
    if (classifier.units.contains("gabor-index"))
    {
        type["gabor"].source["index"] = type["gabor"].load(
                    curvatures[3], classifier.bankClassifiers["gabor"].dict["index"], scaleFactor);
    }
    if (classifier.units.contains("gl-index"))
    {
        type["gl"].source["index"] = type["gl"].load(
                    curvatures[3], classifier.bankClassifiers["gl"].dict["index"], scaleFactor);
    }

    // eigencur
    qDebug() << "eigencur";
    if (classifier.units.contains("gabor-eigencur"))
    {
        type["gabor"].source["eigencur"] = type["gabor"].load(
                    curvatures[4], classifier.bankClassifiers["gabor"].dict["eigencur"], scaleFactor);
    }
    if (classifier.units.contains("gl-eigencur"))
    {
        type["gl"].source["eigencur"] = type["gl"].load(
                    curvatures[4], classifier.bankClassifiers["gl"].dict["eigencur"], scaleFactor);
    }

    // texture
    qDebug() << "texture";
    if (classifier.units.contains("gabor-textureE"))
    {
        type["gabor"].source["textureE"] = type["gabor"].load(
                    texture, classifier.bankClassifiers["gabor"].dict["textureE"], scaleFactor);
    }
    if (classifier.units.contains("gl-textureE"))
    {
        type["gl"].source["textureE"] = type["gl"].load(
                    texture, classifier.bankClassifiers["gl"].dict["textureE"], scaleFactor);
    }

    // isocurves
    qDebug() << "isocurves";
    if (classifier.units.contains("isocurves"))
    {
        VectorOfCurves rawCurves = getIsoGeodesicCurves(properlyAlignedMesh);
        isocurves = classifier.isocurves.extractor.extract(IsoCurveProcessing::generateFeatureVector(rawCurves, false));
    }
}

void Face3DTemplate::deserialize(cv::FileStorage & storage, const FaceClassifier &classifier){
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
		}
	}
}

Face3DTemplate::Face3DTemplate(int id, cv::FileStorage & storage, const FaceClassifier &classifier){
	this->id = id;
	this->deserialize(storage, classifier);
}

Face3DTemplate::Face3DTemplate(int id, const QString &path, const FaceClassifier &classifier)
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

void Face3DTemplate::serialize(cv::FileStorage & storage, const FaceClassifier &classifier) const
{
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

void Face3DTemplate::serialize(const QString &path, const FaceClassifier &classifier) const
{
    cv::FileStorage storage(path.toStdString(), cv::FileStorage::WRITE);
    this->serialize(storage, classifier);
}

/*QVector<Vector> FilterBanksVectors::load(const QString &dirPath, const QString &baseFilename, const FilterBanksClassifiers &classifier)
{
    QStringList srcNames;
    srcNames << "depth" << "index" << "gauss" << "mean" << "eigencur" << "textureE";
    foreach (const QString &srcName, srcNames)
    {
        source[srcName] = load(dirPath, baseFilename, srcName, classifier.dict[srcName]);
    }
}*/

QVector<Vector> FilterBanksVectors::load(const Matrix &image, const FilterBankClassifier &classifier, double scaleFactor)
{
    //static int ii = 0;
    //double min, max;
    //cv::minMaxIdx(image, &min, &max);
    //cv::imwrite((QString("input")+QString::number(ii)+".png").toStdString(), (image-min)/(max-min)*255);
    //qDebug() << min << max;
    //cv::imshow("image", (image-min)/(max-min));
    //cv::waitKey(0);
    //ii++;

    QVector<Vector> result;
    for (int i = 0; i < classifier.realWavelets.count(); i++)
    {
        Matrix response = (classifier.realWavelets[i].rows == 0) ?
                    image :
                    FilterBank::absResponse(image, classifier.realWavelets[i], classifier.imagWavelets[i]);
        Matrix resized = MatrixConverter::scale(response, scaleFactor);

        //qDebug() << resized.rows << resized.cols;
        //cv::minMaxIdx(resized, &min, &max);
        //cv::imwrite((QString::number(ii)+".png").toStdString(), (resized-min)/(max-min)*255);
        //ii++;

        Vector rawVector = MatrixConverter::matrixToColumnVector(resized);
        result << classifier.projections[i].extractor.extract(rawVector);
    }
    return result;
}

/*QVector<Vector> FilterBanksVectors::load(const QString &dirPath, const QString &baseFilename, const QString &source, const FilterBankClassifier &classifier)
{
    Matrix img = MatrixConverter::imageToMatrix(dirPath + source + QDir::separator() + baseFilename + ".png");
    //cv::Rect roi(25, 15, 100, 90);
    //img = img(roi);
    Matrix resized;
    cv::resize(img, resized, cv::Size(img.cols/2, img.rows/2));
    return load(resized, classifier);
}*/
