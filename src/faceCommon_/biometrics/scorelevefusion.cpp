#include "scorelevefusion.h"

#include "linalg/matrixconverter.h"

#include <cassert>

ScoreLevelFusionBase & ScoreLevelFusionBase::addComponent(const Evaluation &component)
{
    components << component;
    learned = false;
	return (*this);
}

void ScoreLevelFusionBase::popComponent()
{
    learned = false;
    components.removeLast();
}

void ScoreLevelFusionBase::learn()
{
    int componentsCount = components.count();
    assert(componentsCount > 1);

    impostorMeans.clear();
    genuineMeans.clear();
    learnImplementation();
	learned = true;
}

Evaluation ScoreLevelFusionBase::evaluate(const QList<Evaluation> &evaluations, bool debugOutput)
{
    assert(learned);

    /*int unitCount = components.count();
    assert(unitCount > 0);
    assert(unitCount == evaluations.count());*/

    int unitCount = evaluations.count();
    assert(unitCount > 0);

    // genuines
    QVector<double> fusedGenuineScores;
    int sameCount = evaluations[0].genuineScores.count();
    for (int i = 0; i < sameCount; i++)
    {
        QVector<double> scores;
        for (int c = 0; c < unitCount; c++)
        {
            scores << evaluations[c].genuineScores[i];
        }
        fusedGenuineScores << fuse(scores);
    }

    // impostors
    QVector<double> fusedImpostorScores;
    int diffCount = evaluations[0].impostorScores.count();
    for (int i = 0; i < diffCount; i++)
    {
        QVector<double> scores;
        for (int c = 0; c < unitCount; c++)
        {
            scores << evaluations[c].impostorScores[i];
        }
        fusedImpostorScores << fuse(scores);
    }

    Evaluation result(fusedGenuineScores, fusedImpostorScores);
    return result;
}

Evaluation ScoreLevelFusionBase::evaluate(const QList<Templates> &templates, const QList<Metrics*> &metrics, bool debugOutput)
{
	assert(learned);

    /*int unitCount = components.count();
    assert(unitCount > 0);
    assert(unitCount == templates.count());
    assert(unitCount == metrics.count());*/

    int unitCount = templates.count();
    assert(unitCount > 0);
    assert(unitCount == metrics.count());

    int n = templates[0].count();

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
            QPair<int, int> pair(templates[0][i].subjectID, templates[0][j].subjectID); // it's not a mistake, we can use index 0 here ;)

            distanceMatrix.insertMulti(pair, distance);
        }
    }

    Evaluation result(distanceMatrix, debugOutput);
    return result;
}

void ScoreLevelFusionBase::prepareDataForClassification(QVector<Vector> &scores, QVector<int> &classes,
                                                        int genuineLabel, int impostorLabel)
{
    scores.clear();
    classes.clear();

    int unitsCount = components.count();
    assert(unitsCount >= 2);

    int genuineCount = components[0].genuineScores.count();
    int impostorCount = components[0].impostorScores.count();

    for (int unit = 0; unit < unitsCount; unit++)
    {
        Vector genuineScoresVec(components[unit].genuineScores);
        double meanGenuine = genuineScoresVec.meanValue();
        Vector impostorScoresVec(components[unit].impostorScores);
        double meanImpostor = impostorScoresVec.meanValue();

        genuineMeans << meanGenuine;
        impostorMeans << meanImpostor;
    }

    for (int i = 0; i < genuineCount; i++)
    {
        QVector<double> scoreVec;
        for (int unit = 0; unit < unitsCount; unit++)
        {
            double s = components[unit].genuineScores[i];
            s = (s - genuineMeans[unit])/(impostorMeans[unit] - genuineMeans[unit]);
            scoreVec << s;
        }
        //qDebug() << "genuine" << scoreVec;

        Vector score(scoreVec);
        scores << score;
        classes << genuineLabel;
    }

    for (int i = 0; i < impostorCount; i++)
    {
        QVector<double> scoreVec;
        for (int unit = 0; unit < unitsCount; unit++)
        {
            double s = components[unit].impostorScores[i];
            s = (s - genuineMeans[unit])/(impostorMeans[unit] - genuineMeans[unit]);
            scoreVec << s;
        }
        //qDebug() << "impostor" << scoreVec;

        Vector score(scoreVec);
        scores << score;
        classes << impostorLabel;
    }
}

Vector ScoreLevelFusionBase::normalizeScore(QVector<double> &score) const
{
    Vector m(score);
    m = m.normalizeComponents(genuineMeans, impostorMeans, false);
    return m;
}

// --- LDA Fusion ---

void ScoreLDAFusion::learnImplementation()
{
    QVector<Vector> scores;
    QVector<int> classes;
    prepareDataForClassification(scores, classes, 0, 1);

    // LDA should find 1 projection vectors (K-1 = 2-1 = 1; K is number of classes (impostors and genuines))
    lda.learn(scores, classes);
    assert(lda.Wt.rows == 1);

    maxScore = -1e300;
    minScore = 1e300;
    int n = components[0].genuineScores.count() + components[0].impostorScores.count();
    for (int i = 0; i < n; i++)
    {
        double s = lda.project(scores[i])(0);
        if (s < minScore) minScore = s;
        if (s > maxScore) maxScore = s;
    }
}

double ScoreLDAFusion::fuse(QVector<double> &scores)
{
    Vector score = normalizeScore(scores);

    double s = lda.project(score)(0);
    if (s > maxScore) return 1.0;
    if (s < minScore) return 0.0;

    double result = (s - minScore)/(maxScore - minScore);
    return result;
}

// --- Logistic regression fusion ---

void ScoreLogisticRegressionFusion::learnImplementation()
{
    QVector<Vector> scores;
    QVector<int> classes;
    prepareDataForClassification(scores, classes, 1, 0);
    logR.learn(scores, classes);
}

double ScoreLogisticRegressionFusion::fuse(QVector<double> &scores) const
{
    Vector normalized = normalizeScore(scores);
    // 1 ~ certainly genuine
    // 0 ~ most likely impostor
    double genuineProbability = logR.classify(normalized);

    // we have to return 'distance', not genuine probability
    double d = 1.0 - genuineProbability;
    //qDebug() << scores << normalized.toQVector() << d;
    return d;
}

// --- Weightes sum Fusion ---

void ScoreWeightedSumFusion::learnImplementation()
{
    int unitsCount = components.count();
    assert(unitsCount >= 2);

    eer.clear();
    weightDenominator = 0.0;
    for (int unit = 0; unit < unitsCount; unit++)
    {
        Vector genuineScoresVec(components[unit].genuineScores);
        double meanGenuine = genuineScoresVec.meanValue();
        Vector impostorScoresVec(components[unit].impostorScores);
        double meanImpostor = impostorScoresVec.meanValue();

        genuineMeans << meanGenuine;
        impostorMeans << meanImpostor;
        eer << components[unit].eer;
        weightDenominator += (0.5 - components[unit].eer);
    }
}

double ScoreWeightedSumFusion::fuse(QVector<double> &scores) const
{
    //qDebug() << scores;
    assert(scores.count() == genuineMeans.count());
    double result = 0.0;
    for (int i = 0; i < scores.count(); i++)
    {
        double w = (0.5-eer[i]) / weightDenominator;
        double s = (scores[i]-genuineMeans[i])/(impostorMeans[i]-genuineMeans[i]);
        result += w*s;
    }
    return result;
}

void ScoreWeightedSumFusion::serialize(const QString &path) const
{
    cv::FileStorage storage(path.toStdString(), cv::FileStorage::WRITE);
    storage << "weightDenominator" << weightDenominator;
    storage << "eer" << Vector(eer);
    storage << "genuineMeans" << Vector(genuineMeans);
    storage << "impostorMeans" << Vector(impostorMeans);
}

ScoreWeightedSumFusion::ScoreWeightedSumFusion(const QString &path)
{
    cv::FileStorage storage(path.toStdString(), cv::FileStorage::READ);
    assert(storage.isOpened());

    storage["weightDenominator"] >> weightDenominator;

    Matrix eerMat;
    storage["eer"] >> eerMat;
    eer = Vector(eerMat).toQVector();

    Matrix genMeansMat;
    storage["genuineMeans"] >> genMeansMat;
    genuineMeans = Vector(genMeansMat).toQVector();

    Matrix impMeansMat;
    storage["impostorMeans"] >> impMeansMat;
    impostorMeans = Vector(impMeansMat).toQVector();

    learned = true;
}

// --- Product rule fusion ---

void ScoreProductFusion::learnImplementation()
{
    int unitsCount = components.count();
    assert(unitsCount >= 2);

    for (int unit = 0; unit < unitsCount; unit++)
    {
        Vector genuineScoresVec(components[unit].genuineScores);
        double meanGenuine = genuineScoresVec.meanValue();
        genuineMeans << meanGenuine;
        Vector impostorScoresVec(components[unit].impostorScores);
        double meanImpostor = impostorScoresVec.meanValue();
        impostorMeans << meanImpostor;
    }
}

double ScoreProductFusion::fuse(QVector<double> &scores) const
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

void ScoreSVMFusion::learnImplementation()
{
    QVector<Vector> scores;
    QVector<int> classes;
    prepareDataForClassification(scores, classes, 1, -1);

    cv::Mat data = colVectorsToFPMatrix(scores).t();
    cv::Mat labels = colVectorToColFPMatrix(classes);

    cv::SVMParams params;
    params.svm_type = cv::SVM::C_SVC;
    params.kernel_type = cv::SVM::LINEAR;
    params.term_crit   = cv::TermCriteria(CV_TERMCRIT_ITER, 1000, 1e-6);
    svm.train(data, labels, cv::Mat(), cv::Mat(), params);
}

void ScoreSVMFusion::serialize(const QString &path)
{
    svm.save(path.toStdString().c_str(), "svm");

    cv::FileStorage storage(path.toStdString(), cv::FileStorage::APPEND);
    storage << "genuineMeans" << Vector(genuineMeans);
    storage << "impostorMeans" << Vector(impostorMeans);
}

ScoreSVMFusion::ScoreSVMFusion(const QString &path)
{
    svm.load(path.toStdString().c_str(), "svm");

    cv::FileStorage storage(path.toStdString(), cv::FileStorage::READ);
    assert(storage.isOpened());

    Matrix genMeansMat;
    storage["genuineMeans"] >> genMeansMat;
    genuineMeans = Vector(genMeansMat).toQVector();

    Matrix impMeansMat;
    storage["impostorMeans"] >> impMeansMat;
    impostorMeans = Vector(impMeansMat).toQVector();

    learned = true;
}

double ScoreSVMFusion::fuse(QVector<double> &scores) const
{
    //qDebug() << scores;

    assert(scores.count() == genuineMeans.count());
    for (int i = 0; i < scores.count(); i++)
        scores[i] = (scores[i]-genuineMeans[i])/(impostorMeans[i]-genuineMeans[i]);

    //qDebug() << scores;

    cv::Mat score = colVectorToColFPMatrix(scores);

    double svmResult = svm.predict(score, true);

    //qDebug() << svmResult;
    return svmResult;
}

cv::Mat ScoreSVMFusion::colVectorToColFPMatrix(QVector<int> &vector) const
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

cv::Mat ScoreSVMFusion::colVectorToColFPMatrix(QVector<double> &vector) const
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

cv::Mat ScoreSVMFusion::colVectorsToFPMatrix(QVector<Vector> &vectors) const
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
