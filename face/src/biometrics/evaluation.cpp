#include "evaluation.h"

#include <QDebug>

#include <cmath>

Evaluation::Evaluation(QHash<QPair<int, int>, double> &distances, bool debugOutput)
{
    if (debugOutput)
        qDebug() << "Evaluation";

    minSameDistance = 1e300;
    minDifferentDistance = 1e300;
    minDistance = 1e300;

    maxSameDistance = 0.0;
    maxDifferentDistance = 0.0;
    maxDistance = 0.0;

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
            scores.append(d);

            if (d > maxDistance) maxDistance = d;
            if (d < minDistance) minDistance = d;
        }
    }

    commonEvaluation(debugOutput);
}

Evaluation::Evaluation(QVector<Template> &templates, Metrics &metrics, bool debugOutput)
{
    if (debugOutput)
        qDebug() << "Evaluation";

    commonTemplatesEvaluation(templates, metrics, debugOutput);
    commonEvaluation(debugOutput);
}

Evaluation::Evaluation(QVector<Matrix> &rawData, QVector<int> &classes,
                       FeatureExtractor &extractor, Metrics &metric, bool debugOutput)
{
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

        // Impostor, Genuine
        // TODO
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

void Evaluation::commonTemplatesEvaluation(QVector<Template> &templates, Metrics &metrics, bool debugOutput)
{
    minSameDistance = 1e300;
    minDifferentDistance = 1e300;
    minDistance = 1e300;

    maxSameDistance = -1e300;
    maxDifferentDistance = -1e300;
    maxDistance = -1e300;

    if (debugOutput)
        qDebug() << "  distance calculation";
    int n = templates.count();
    for (int i = 0; i < (n-1); i++)
    {
        Template &ti = templates[i];
        assert(!(Common::matrixContainsNan(ti.featureVector)));

        for (int j = i+1; j < n; j++)
        {
            Template &tj = templates[j];
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
            scores.append(d);

            if (d > maxDistance) maxDistance = d;
            if (d < minDistance) minDistance = d;
        }
    }

    //assert(maxDifferentDistance > maxSameDistance); // FIXME: not sure
    //assert(minSameDistance < minDifferentDistance); // FIXME: not sure
    assert(maxSameDistance > minSameDistance);
    assert(maxDifferentDistance > minDifferentDistance);
}

void Evaluation::outputResultsDET(const QString &path)
{
    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    for (int i = 0; i < thresholds.count(); i++)
    {
        out << fmr[i] << ' ' << fnmr[i] << '\n';
    }
}

void Evaluation::outputResultsGenuine(const QString &path)
{
    // TODO
}

void Evaluation::outputResultsImpostor(const QString &path)
{
    // TODO
}

void Evaluation::outputResults(const QString &path)
{
    // DET
    outputResultsDET(path+"-det");

    // Impostor, Genuine
    outputResultsImpostor(path+"-impostor");
    outputResultsGenuine(path+"-genuine");
}

BatchEvaluationResult Evaluation::batch(QList<QVector<Template> > &templates, Metrics &metrics, int startIndex)
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

    batchResult.meanEER = Vector::meanValue(eer);
    batchResult.stdDevOfEER = Vector::stdDeviation(eer);

    return batchResult;
}

BatchEvaluationResult Evaluation::batch(QList<QVector<Matrix> > &images, QList<QVector<int> > &classes,
		FeatureExtractor &extractor, Metrics &metrics, int startIndex)
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

    batchResult.meanEER = Vector::meanValue(eer);
    batchResult.stdDevOfEER = Vector::stdDeviation(eer);

    return batchResult;
}
