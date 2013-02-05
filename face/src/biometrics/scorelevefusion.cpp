#include "scorelevefusion.h"

#include "linalg/matrixconverter.h"

#include <cassert>

ScoreLevelFusionBase & ScoreLevelFusionBase::addComponent(
		QVector<Matrix> &trainRawData,
		QVector<int> &trainClasses,
		FeatureExtractor &featureExtractor,
		Metrics &metrics)
{
	this->trainClasses << (&trainClasses);
	this->trainRawData << (&trainRawData);
	this->extractors << (&featureExtractor);
	this->metrics << (&metrics);

	return (*this);
}

void ScoreLevelFusionBase::learn()
{
	int unitsCount = trainClasses.count();
	assert(unitsCount > 1);

	QList<Evaluation> evaluationResults;
	for (int i = 0; i < unitsCount; i++)
	{
		Evaluation e(*(trainRawData[i]), *(trainClasses[i]), *(extractors[i]), *(metrics[i]));
		evaluationResults << e;
	}
	learnImplementation(evaluationResults);
	learned = true;
}

Evaluation ScoreLevelFusionBase::evaluate(QList<QVector<Matrix> > &rawData, QVector<int> &classes, bool debugOutput)
{
	assert(learned);

    int unitCount = extractors.count();
    assert(unitCount > 0);
    assert(unitCount == rawData.count());

    int n = rawData[0].count();
    assert(n == classes.count());

    QVector<QVector<Template> > templates;
    for (int unit = 0; unit < unitCount; unit++)
    {
        FeatureExtractor &extractor = *(extractors[unit]);
        Templates t = Template::createTemplates(rawData[unit], classes, extractor);
        templates.append(t);
    }

    QHash<QPair<int, int>, double> distanceMatrix;
    for (int i = 0; i < (n-1); i++)
    {
        for (int j = i+1; j < n; j++)
        {
            QVector<double> distances;
            for (int unit = 0; unit < unitCount; unit++)
            {
                double d = metrics[unit]->distance(templates[unit][i].featureVector, templates[unit][j].featureVector);
                assert(d == d);
                distances << d;
            }
            double distance = fuse(distances);
            QPair<int, int> pair(templates[0][i].subjectID, templates[0][j].subjectID);

            distanceMatrix.insertMulti(pair, distance);
        }
    }

    Evaluation result(distanceMatrix, debugOutput);
    return result;
}

void ScoreLevelFusionBase::prepareDataForClassification(QList<Evaluation> &evaluationResults,
                                                  QVector<Matrix> &scores, QVector<int> &classes,
                                                  int genuineLabel, int impostorLabel)
{
    scores.clear();
    classes.clear();
    impostorMeans.clear();
    genuineMeans.clear();

    int unitsCount = evaluationResults.count();
    assert(unitsCount >= 2);

    int genuineCount = evaluationResults[0].genuineScores.count();
    int impostorCount = evaluationResults[0].impostorScores.count();

    for (int unit = 0; unit < unitsCount; unit++)
    {
        double meanGenuine = Vector::meanValue(evaluationResults[unit].genuineScores);
        double meanImpostor = Vector::meanValue(evaluationResults[unit].impostorScores);

        genuineMeans << meanGenuine;
        impostorMeans << meanImpostor;
    }

    for (int i = 0; i < genuineCount; i++)
    {
        QVector<double> scoreVec;
        for (int unit = 0; unit < unitsCount; unit++)
        {
            double s = evaluationResults[unit].genuineScores[i];
            s = (s - genuineMeans[unit])/(impostorMeans[unit] - genuineMeans[unit]);
            scoreVec << s;
        }

        Matrix score = Vector::fromQVector(scoreVec);
        scores << score;
        classes << genuineLabel;
    }

    for (int i = 0; i < impostorCount; i++)
    {
        QVector<double> scoreVec;
        for (int unit = 0; unit < unitsCount; unit++)
        {
            double s = evaluationResults[unit].impostorScores[i];
            s = (s - genuineMeans[unit])/(impostorMeans[unit] - genuineMeans[unit]);
            scoreVec << s;
        }

        Matrix score = Vector::fromQVector(scoreVec);
        scores << score;
        classes << impostorLabel;
    }
}

Matrix ScoreLevelFusionBase::normalizeScore(QVector<double> &score)
{
    Matrix m = Vector::fromQVector(score);
    m = Vector::normalizeComponents(m, impostorMeans, genuineMeans, false);
    return m;
}

// --- LDA Fusion ---

void ScoreLDAFusion::learnImplementation(QList<Evaluation> &evaluationResults)
{
    QVector<Matrix> scores;
    QVector<int> classes;
    prepareDataForClassification(evaluationResults, scores, classes, 0, 1);

    // LDA should find 1 projection vectors (K-1 = 2-1 = 1; K is number of classes (impostors and genuines))
    lda.learn(scores, classes, false);
    assert(lda.Wt.rows == 1);

    maxScore = -1e300;
    minScore = 1e300;
    int n = evaluationResults[0].scores.count(); //genuineCount+impostorCount;
    for (int i = 0; i < n; i++)
    {
        double s = lda.project(scores[i])(0);
        if (s < minScore) minScore = s;
        if (s > maxScore) maxScore = s;
    }
}

double ScoreLDAFusion::fuse(QVector<double> &scores)
{
    Matrix score = normalizeScore(scores);

    double s = lda.project(score)(0);
    if (s > maxScore) return 1.0;
    if (s < minScore) return 0.0;

    double result = 1-(s - minScore)/(maxScore - minScore);
    return result;
}

// --- Logistic regression fusion ---

void ScoreLogisticRegressionFusion::learnImplementation(QList<Evaluation> &evaluationResults)
{
    QVector<Matrix> scores;
    QVector<int> classes;
    prepareDataForClassification(evaluationResults, scores, classes, 0, 1);

    logR.learn(scores, classes);
}

double ScoreLogisticRegressionFusion::fuse(QVector<double> &scores)
{
    Matrix score = normalizeScore(scores);
    double genuineProbability = logR.classify(score);

    // 0 ~ certainly genuine (we have to return distance)
    // 1 ~ most likely impostor
    return 1.0 - genuineProbability;
}

// --- Weightes sum Fusion ---

void ScoreWeightedSumFusion::learnImplementation(QList<Evaluation> &evaluationResults)
{
    int unitsCount = evaluationResults.count();
    assert(unitsCount >= 2);

    weightDenominator = 0.0;
    for (int unit = 0; unit < unitsCount; unit++)
    {
        double meanGenuine = Vector::meanValue(evaluationResults[unit].genuineScores);
        double meanImpostor = Vector::meanValue(evaluationResults[unit].impostorScores);

        genuineMeans << meanGenuine;
        impostorMeans << meanImpostor;
        eer << evaluationResults[unit].eer;
        weightDenominator += (1-evaluationResults[unit].eer);
    }
}

double ScoreWeightedSumFusion::fuse(QVector<double> &scores)
{
    assert(scores.count() == genuineMeans.count());
    double result = 0.0;
    for (int i = 0; i < scores.count(); i++)
    {
        double w = (1-eer[i]) / weightDenominator;
        double s = (scores[i]-genuineMeans[i])/(impostorMeans[i]-genuineMeans[i]);
        result += w*s;
    }
    return result;
}

// --- Product rule fusion ---

void ScoreProductFusion::learnImplementation(QList<Evaluation> &evaluationResults)
{
    int unitsCount = evaluationResults.count();
    assert(unitsCount >= 2);

    for (int unit = 0; unit < unitsCount; unit++)
    {
        genuineMeans << Vector::meanValue(evaluationResults[unit].genuineScores);
        impostorMeans << Vector::meanValue(evaluationResults[unit].impostorScores);
    }
}

double ScoreProductFusion::fuse(QVector<double> &scores)
{
    assert(scores.count() == genuineMeans.count());
    double result = 1.0;
    for (int i = 0; i < scores.count(); i++)
    {
        double s = (scores[i]-genuineMeans[i])/(impostorMeans[i]-genuineMeans[i]);
        if (s > 1) s = 1;
        if (s < 0) s = 0;
        result *= s;
    }
    return result;
}

// --- SVM Fusion ---

void ScoreSVMFusion::learnImplementation(QList<Evaluation> &evaluationResults)
{
    QVector<Matrix> scores;
    QVector<int> classes;
    prepareDataForClassification(evaluationResults, scores, classes, -1, 1);

    Matrix data = colVectorsToFPMatrix(scores).t();
    Matrix labels = colVectorToColFPMatrix(classes);

    cv::SVMParams params;
    params.svm_type = cv::SVM::C_SVC;
    params.kernel_type = cv::SVM::SIGMOID;
    params.term_crit   = cv::TermCriteria(CV_TERMCRIT_ITER, 1000, 1e-6);
    svm.train(data, labels, Matrix(), Matrix(), params);
}

double ScoreSVMFusion::fuse(QVector<double> &scores)
{
    assert(scores.count() == genuineMeans.count());
    for (int i = 0; i < scores.count(); i++)
        scores[i] = (scores[i]-genuineMeans[i])/(impostorMeans[i]-genuineMeans[i]);

    cv::Mat score = colVectorToColFPMatrix(scores);
    double svmResult = svm.predict(score, true);
    return svmResult;
}

cv::Mat ScoreSVMFusion::colVectorToColFPMatrix(QVector<int> &vector)
{
    int r = vector.count();
    cv::Mat result = cv::Mat::zeros(r, 1, CV_32F);
    for (int i = 0; i < r; i++)
    {
        float val = vector[i];
        result.at<float>(i) = val;
    }
    return result;
}

cv::Mat ScoreSVMFusion::colVectorToColFPMatrix(QVector<double> &vector)
{
    int r = vector.count();
    cv::Mat result = cv::Mat::zeros(r, 1, CV_32F);
    for (int i = 0; i < r; i++)
    {
        float val = vector[i];
        result.at<float>(i) = val;
    }
    return result;
}

cv::Mat ScoreSVMFusion::colVectorsToFPMatrix(QVector<Matrix> &vectors)
{
    int cols = vectors.count();
    int rows = vectors[0].rows;

    assert(rows > 0);
    assert(cols > 0);

    cv::Mat data(rows, cols, CV_32F);
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            float val = vectors[c](r);
            data.at<float>(r,c) = val;
        }
    }

    return data;
}
