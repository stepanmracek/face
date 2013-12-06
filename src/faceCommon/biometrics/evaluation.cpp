#include "evaluation.h"

#include <QDebug>
#include <cmath>

#include "linalg/histogram.h"

void Evaluation::commonInit()
{
    eer = 1.0;
    eerDistance = 0.0;

    minSameDistance = 1e300;
    minDifferentDistance = 1e300;
    minDistance = 1e300;

    maxSameDistance = -1e300;
    maxDifferentDistance = -1e300;
    maxDistance = -1e300;
}

Evaluation::Evaluation(const QVector<double> &genuineScores, const QVector<double> &impostorScores, bool debugOutput) :
    genuineScores(genuineScores), impostorScores(impostorScores)
{
    commonInit();

    if (debugOutput)
        qDebug() << "Evaluation";

    Vector genVec(genuineScores);
    minSameDistance = genVec.minValue();
    maxSameDistance = genVec.maxValue();

    Vector impVec(impostorScores);
    minDifferentDistance = impVec.minValue();
    maxDifferentDistance = impVec.maxValue();

    minDistance = minSameDistance;
    maxDistance = maxDifferentDistance;

    if (debugOutput)
    {
        qDebug() << "  same distance range:" << minSameDistance << maxSameDistance;
        qDebug() << "  different distance range" << minDifferentDistance << maxDifferentDistance;
    }

    assert((maxSameDistance > minSameDistance) && (maxDifferentDistance > minDifferentDistance));

    commonEvaluation(debugOutput);
}

Evaluation::Evaluation(QHash<QPair<int, int>, double> &distances, bool debugOutput)
{
    commonInit();

    if (debugOutput)
        qDebug() << "Evaluation";

    if (debugOutput)
        qDebug() << "  distance calculation";

    QList< QPair<int, int> > pairs = distances.uniqueKeys();
    int numPairs = pairs.count();
    for (int i = 0; i < numPairs; i++)
    {
        QPair<int, int> pair = pairs[i];

        QList<double> values = distances.values(pair);
        for (int i = 0; i < values.count(); i++)
        {
            double d = values[i];
            assert(d == d); // Not NaN

            // same or different?
            if (pair.first == pair.second)
            {
                genuineScores.append(d);
                if (d > maxSameDistance) maxSameDistance = d;
                if (d < minSameDistance) minSameDistance = d;
            }
            else
            {
                impostorScores.append(d);
                if (d > maxDifferentDistance) maxDifferentDistance = d;
                if (d < minDifferentDistance) minDifferentDistance = d;
            }

            if (d > maxDistance) maxDistance = d;
            if (d < minDistance) minDistance = d;
        }
    }

    if (debugOutput)
    {
        qDebug() << "  same distance range:" << minSameDistance << maxSameDistance;
        qDebug() << "  different distance range" << minDifferentDistance << maxDifferentDistance;
    }

    commonEvaluation(debugOutput);
}

Evaluation::Evaluation(const QVector<Template> &templates, const Metrics &metrics, bool debugOutput)
{
    commonInit();

    if (debugOutput)
        qDebug() << "Evaluation";

    commonTemplatesEvaluation(templates, metrics, debugOutput);
    commonEvaluation(debugOutput);
}

Evaluation::Evaluation(const QVector<Vector> &rawData, const QVector<int> &classes,
                       const FeatureExtractor &extractor, const Metrics &metric, bool debugOutput)
{
    commonInit();
    QVector<Template> templates = Template::createTemplates(rawData, classes, extractor);

    if (debugOutput)
        qDebug() << "Evaluation";

    commonTemplatesEvaluation(templates, metric, debugOutput);
    commonEvaluation(debugOutput);
}

void Evaluation::commonEvaluation(bool debugOutput)
{
    // DET, EER
    if (debugOutput)
        qDebug() << "  DET and EER calculation";
    double delta = maxDistance - minDistance;
    double step = delta/1000.0;
    if (step == 0) return;

    double minDiff = 1e300;

    for (double t = minDistance; t <= maxDistance; t += step)
    {
        thresholds.append(t);

        // FNMR
        double sameMismatchCount = 0.0;
        for (int i = 0; i < genuineScores.count(); i++)
            if (genuineScores[i] > t) sameMismatchCount+=1.0;
        sameMismatchCount /= genuineScores.count();
        fnmr.append(sameMismatchCount);

        // FMR
        double differentMatchCount = 0.0;
        for (int i = 0; i < impostorScores.count(); i++)
            if (impostorScores[i] < t) differentMatchCount+=1.0;
        differentMatchCount /= impostorScores.count();
        fmr.append(differentMatchCount);

        // EER
        double diff = fabs(sameMismatchCount - differentMatchCount);
        if (diff < minDiff)
        {
            minDiff = diff;
            eer = (sameMismatchCount+differentMatchCount)/2;
            eerDistance = t;
        }
    }
}

double Evaluation::fnmrAtFmr(double _fmr)
{
	int n = fmr.count();
	assert(n > 0);
	assert(n == fnmr.count());

	double minDiff = 1e300;
	double _fnmr = 1.0;
	for (int i = 0; i < n; i++)
	{
		double diff = fabs(fmr[i]-_fmr);
		if (diff < minDiff)
		{
			minDiff = diff;
			_fnmr = fnmr[i];
		}
	}

	return _fnmr;
}

void Evaluation::commonTemplatesEvaluation(const QVector<Template> &templates, const Metrics &metrics, bool debugOutput)
{
    if (debugOutput)
        qDebug() << "  distance calculation";
    int n = templates.count();
    for (int i = 0; i < (n-1); i++)
    {
        const Template &ti = templates[i];
        assert(!(Common::matrixContainsNan(ti.featureVector)));

        for (int j = i+1; j < n; j++)
        {
            const Template &tj = templates[j];
            assert(!(Common::matrixContainsNan(tj.featureVector)));
            double d = metrics.distance(ti.featureVector, tj.featureVector);
            assert(d == d); // Not NaN

            // same or different?
            if (ti.subjectID == tj.subjectID)
            {
                genuineScores.append(d);
                if (d > maxSameDistance) maxSameDistance = d;
                if (d < minSameDistance) minSameDistance = d;
            }
            else
            {
                impostorScores.append(d);
                if (d > maxDifferentDistance) maxDifferentDistance = d;
                if (d < minDifferentDistance) minDifferentDistance = d;
            }

            if (d > maxDistance) maxDistance = d;
            if (d < minDistance) minDistance = d;
        }
    }

    if (debugOutput)
    {
        qDebug() << "  same distance range:" << minSameDistance << maxSameDistance;
        qDebug() << "  different distance range" << minDifferentDistance << maxDifferentDistance;
    }

    assert((maxSameDistance > minSameDistance) && (maxDifferentDistance > minDifferentDistance));
}

void Evaluation::outputResultsDET(const QString &path) const
{
    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    for (int i = 0; i < thresholds.count(); i++)
    {
        out << fmr[i] << ' ' << fnmr[i] << '\n';
    }
}

void Evaluation::outputResultsFMR(const QString &path) const
{
    Common::savePlot(thresholds, fmr, path);
}

void Evaluation::outputResultsFNMR(const QString &path) const
{
    Common::savePlot(thresholds, fnmr, path);
}

void Evaluation::outputResultsGenuineDistribution(const QString &path, int bins) const
{
    Histogram(genuineScores, bins, true, minDistance, maxDistance).savePlot(path);
}

void Evaluation::outputResultsGenuineScores(const QString &path) const
{
    Vector(genuineScores).toFile(path);
}

void Evaluation::outputResultsImpostorDistribution(const QString &path, int bins) const
{
    Histogram(impostorScores, bins, true, minDistance, maxDistance).savePlot(path);
}

void Evaluation::outputResultsImpostorScores(const QString &path) const
{
    Vector(impostorScores).toFile(path);
}

void Evaluation::outputResults(const QString &path, int histogramBins) const
{
    // DET
    outputResultsFMR(path+"-fmr");
    outputResultsFNMR(path+"-fnmr");
    outputResultsDET(path+"-det");

    // Impostor, Genuine
    outputResultsImpostorDistribution(path+"-imp-distrib", histogramBins);
    outputResultsImpostorScores(path+"-imp-scores");
    outputResultsGenuineDistribution(path+"-gen-distrib", histogramBins);
    outputResultsGenuineScores(path+"-gen-scores");
}

BatchEvaluationResult Evaluation::batch(QList<QVector<Template> > &templates, const Metrics &metrics, int startIndex)
{
    BatchEvaluationResult batchResult;
    QVector<double> eer;

    int cCount = templates.count();
    for (int i = startIndex; i < cCount; i++)
    {
        Evaluation e(templates[i], metrics, false);
        batchResult.results.append(e);
        eer.append(e.eer);
    }

    Vector eerVec = Vector(eer);
    batchResult.meanEER = eerVec.meanValue();
    batchResult.stdDevOfEER = eerVec.stdDeviation();

    return batchResult;
}

BatchEvaluationResult Evaluation::batch(QList<QVector<Vector> > &images, QList<QVector<int> > &classes,
        const FeatureExtractor &extractor, const Metrics &metrics, int startIndex)
{
    BatchEvaluationResult batchResult;
    QVector<double> eer;

    int cCount = images.count();
    for (int i = startIndex; i < cCount; i++)
    {
        Evaluation e(images[i], classes[i], extractor, metrics, false);
        batchResult.results.append(e);
        eer.append(e.eer);
    }

    Vector eerVec = Vector(eer);
    batchResult.meanEER = eerVec.meanValue();
    batchResult.stdDevOfEER = eerVec.stdDeviation();

    return batchResult;
}
