#include "faceCommon/biometrics/scorelevefusion.h"

#include <Poco/StringTokenizer.h>

#include "faceCommon/linalg/matrixconverter.h"
#include "faceCommon/linalg/vector.h"
#include "faceCommon/biometrics/template.h"

using namespace Face::Biometrics;

ScoreLevelFusionBase & ScoreLevelFusionBase::addComponent(const Evaluation &component)
{
    components.push_back(component);
    learned = false;
	return (*this);
}

void ScoreLevelFusionBase::popComponent()
{
    learned = false;
    components.pop_back();
}

void ScoreLevelFusionBase::learn()
{
    int componentsCount = components.size();
    if (componentsCount < 2) throw FACELIB_EXCEPTION("fusion should contain at least two units");

    scoreNormalizer->learn(components);
    learnImplementation();
	learned = true;
}

Evaluation ScoreLevelFusionBase::evaluate(const std::vector<Evaluation> &evaluations)
{
    if(!learned) throw FACELIB_EXCEPTION("not learned");

    int unitCount = evaluations.size();
    if (unitCount == 0) throw FACELIB_EXCEPTION("empty input evaluations list");

    // genuines
    std::vector<double> fusedGenuineScores;
    int sameCount = evaluations[0].genuineScores.size();
    for (int i = 0; i < sameCount; i++)
    {
        std::vector<double> scores;
        for (int c = 0; c < unitCount; c++)
        {
            scores.push_back(evaluations[c].genuineScores[i]);
        }
        fusedGenuineScores.push_back(fuse(scores).score);
    }

    // impostors
    std::vector<double> fusedImpostorScores;
    int diffCount = evaluations[0].impostorScores.size();
    for (int i = 0; i < diffCount; i++)
    {
        std::vector<double> scores;
        for (int c = 0; c < unitCount; c++)
        {
            scores.push_back(evaluations[c].impostorScores[i]);
        }
        fusedImpostorScores.push_back(fuse(scores).score);
    }

    Evaluation result(fusedGenuineScores, fusedImpostorScores);
    return result;
}

Evaluation ScoreLevelFusionBase::evaluate(const std::vector<std::vector<Face::Biometrics::Template> > &templates,
                                          const std::vector<Face::LinAlg::Metrics*> &metrics)
{
    if (!learned) throw FACELIB_EXCEPTION("not learned");

    unsigned int unitCount = templates.size();
    if (unitCount == 0) throw FACELIB_EXCEPTION("empty templates count");
    if (unitCount != metrics.size()) throw FACELIB_EXCEPTION("units and metrics count mismatch");

    unsigned int n = templates[0].size();

    std::map<std::pair<int, int>, std::vector<double> > distanceMatrix;
    for (unsigned int i = 0; i < (n-1); i++)
    {
        for (unsigned int j = i+1; j < n; j++)
        {
            std::vector<double> distances;
            for (unsigned int unit = 0; unit < unitCount; unit++)
            {
                double d = metrics[unit]->distance(templates[unit][i].featureVector, templates[unit][j].featureVector);
                if (d != d) throw FACELIB_EXCEPTION("NaN");
                distances.push_back(d);
            }
            double distance = fuse(distances).score;

            // it's not a mistake, we can use index 0 here ;)
            distanceMatrix[std::make_pair(templates[0][i].subjectID, templates[0][j].subjectID)].push_back(distance);
        }
    }

    Evaluation result(distanceMatrix);
    return result;
}

void ScoreLevelFusionBase::prepareDataForClassification(std::vector<Face::LinAlg::Vector> &scores, std::vector<int> &classes,
                                                        int genuineLabel, int impostorLabel)
{
    scores.clear();
    classes.clear();

    int unitsCount = components.size();
    if (unitsCount < 2) throw FACELIB_EXCEPTION("there should be at least two units");

    int genuineCount = components[0].genuineScores.size();
    int impostorCount = components[0].impostorScores.size();

    for (int i = 0; i < genuineCount; i++)
    {
        std::vector<double> scoreVec;
        for (int unit = 0; unit < unitsCount; unit++)
        {
            scoreVec.push_back(components[unit].genuineScores[i]);
        }

        Face::LinAlg::Vector score(scoreNormalizer->normalize(scoreVec));
        scores.push_back(score);
        classes.push_back(genuineLabel);
    }

    for (int i = 0; i < impostorCount; i++)
    {
        std::vector<double> scoreVec;
        for (int unit = 0; unit < unitsCount; unit++)
        {
            scoreVec.push_back(components[unit].impostorScores[i]);
        }

        Face::LinAlg::Vector score(scoreNormalizer->normalize(scoreVec));
        scores.push_back(score);
        classes.push_back(impostorLabel);
    }
}

ScoreLevelFusionBase::Ptr ScoreLevelFusionFactory::create(const std::string &name)
{
    Poco::StringTokenizer items(name, "-");
    if (items.count() != 2)
        throw FACELIB_EXCEPTION("unknown fusion " + name);

    ScoreLevelFusionBase *result;
    if (items[0] == ScoreLDAFusion::name())
        result = new ScoreLDAFusion();
    else if (items[0] == ScoreLogisticRegressionFusion::name())
        result = new ScoreLogisticRegressionFusion();
    else if (items[0] == ScoreWeightedSumFusion::name())
        result = new ScoreWeightedSumFusion();
    else if (items[0] == ScoreProductFusion::name())
        result = new ScoreProductFusion();
    else if (items[0] == ScoreSVMFusion::name())
        result = new ScoreSVMFusion();
    else if (items[0] == ScoreGMMFusion::name())
        result = new ScoreGMMFusion();
    else if (items[0] == ScoreSumFusion::name())
        result = new ScoreSumFusion();
    else
        throw FACELIB_EXCEPTION("unknown fusion " + name);

    result->scoreNormalizer = ScoreNormalizerFactory::create(items[1]);
    return result;
}

// --- LDA Fusion ---

void ScoreLDAFusion::learnImplementation()
{
    std::vector<Face::LinAlg::Vector> scores;
    std::vector<int> classes;
    prepareDataForClassification(scores, classes, 0, 1);

    // LDA should find 1 projection vectors (K-1 = 2-1 = 1; K is number of classes (impostors and genuines))
    lda.learn(scores, classes);

    /*maxScore = -1e300;
    minScore = 1e300;
    int n = components[0].genuineScores.count() + components[0].impostorScores.count();
    for (int i = 0; i < n; i++)
    {
        double s = lda.project(scores[i])(0);
        if (s < minScore) minScore = s;
        if (s > maxScore) maxScore = s;
    }*/
}

ScoreLevelFusionBase::Result ScoreLDAFusion::fuse(const std::vector<double> &scores) const
{
    Result result;
    result.preNormalized = scores;
    result.normalized = scoreNormalizer->normalize(scores);
    result.score = lda.project(result.normalized)(0);
    return result;
}

void ScoreLDAFusion::serialize(const std::string &path) const
{
    lda.serialize(path);

    cv::FileStorage storage(path, cv::FileStorage::APPEND);
    scoreNormalizer->serialize(storage);
}

void ScoreLDAFusion::deserialize(const std::string &path)
{
    lda = Face::LinAlg::LDA(path);

    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened())
    {
        throw FACELIB_EXCEPTION("can't read file " + path);
    }

    scoreNormalizer->deserialize(storage);
    learned = true;
}

// --- Logistic regression fusion ---

void ScoreLogisticRegressionFusion::learnImplementation()
{
    std::vector<Face::LinAlg::Vector> scores;
    std::vector<int> classes;
    prepareDataForClassification(scores, classes, 1, 0);
    logR.learn(scores, classes);
}

ScoreLevelFusionBase::Result ScoreLogisticRegressionFusion::fuse(const std::vector<double> &scores) const
{
    Result result;
    result.preNormalized = scores;
    result.normalized = scoreNormalizer->normalize(scores);

    // 1 ~ certainly genuine
    // 0 ~ most likely impostor
    double genuineProbability = logR.classify(result.normalized);

    // we have to return 'distance', not genuine probability
    result.score = 1.0 - genuineProbability;
    return result;
}

void ScoreLogisticRegressionFusion::serialize(const std::string &path) const
{
    logR.serialize(path);

    cv::FileStorage storage(path, cv::FileStorage::APPEND);
    scoreNormalizer->serialize(storage);
}

void ScoreLogisticRegressionFusion::deserialize(const std::string &path)
{
    logR = Face::LinAlg::LogisticRegression(path);

    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened())
    {
        throw FACELIB_EXCEPTION("can't read file " + path);
    }

    scoreNormalizer->deserialize(storage);

    learned = true;
}

// --- Weighted sum Fusion ---

void ScoreWeightedSumFusion::learnImplementation()
{
    int unitsCount = components.size();
    if (unitsCount < 2) throw FACELIB_EXCEPTION("there should be at least two units");

    eer.clear();
    weightDenominator = 0.0;
    for (int unit = 0; unit < unitsCount; unit++)
    {
        eer.push_back(components[unit].eer);
        weightDenominator += (0.5 - components[unit].eer);
    }
}

ScoreLevelFusionBase::Result ScoreWeightedSumFusion::fuse(const std::vector<double> &scores) const
{
    Result result;
    result.preNormalized = scores;
    result.normalized = scoreNormalizer->normalize(scores);

    for (unsigned int i = 0; i < scores.size(); i++)
    {
        double w = (0.5 - eer[i]) / weightDenominator;
        result.score += w * result.normalized[i];
    }
    return result;
}

void ScoreWeightedSumFusion::serialize(const std::string &path) const
{
    cv::FileStorage storage(path, cv::FileStorage::WRITE);
    storage << "weightDenominator" << weightDenominator;
    storage << "eer" << Face::LinAlg::Vector(eer);

    scoreNormalizer->serialize(storage);
}

void ScoreWeightedSumFusion::deserialize(const std::string &path)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened())
    {
        throw FACELIB_EXCEPTION("can't read file " + path);
    }

    storage["weightDenominator"] >> weightDenominator;

    Face::LinAlg::Vector eerMat;
    storage["eer"] >> eerMat;
    eer = eerMat.toStdVector();

    scoreNormalizer->deserialize(storage);

    learned = true;
}

// --- Sum rule fusion ---

void ScoreSumFusion::learnImplementation()
{

}

ScoreLevelFusionBase::Result ScoreSumFusion::fuse(const std::vector<double> &scores) const
{
    Result result;
    result.preNormalized = scores;
    result.normalized = scoreNormalizer->normalize(scores);

    int n = scores.size();
    for (int i = 0; i < n; i++)
    {
       result.score += result.normalized[i];
    }
    return result;
}

void ScoreSumFusion::serialize(const std::string &/*path*/) const
{
    throw FACELIB_EXCEPTION("not implemented");
}

void ScoreSumFusion::deserialize(const std::string &/*path*/)
{
    throw FACELIB_EXCEPTION("not implemented");
}

// --- Product rule fusion ---

void ScoreProductFusion::learnImplementation()
{

}

ScoreLevelFusionBase::Result ScoreProductFusion::fuse(const std::vector<double> &scores) const
{
    Result result;
    result.preNormalized = scores;
    result.normalized = scoreNormalizer->normalize(scores);

    for (unsigned int i = 0; i < scores.size(); i++)
    {
        result.score *= result.normalized[i];
    }
    return result;
}

void ScoreProductFusion::serialize(const std::string &path) const
{
    cv::FileStorage storage(path, cv::FileStorage::WRITE);
    scoreNormalizer->serialize(storage);
}

void ScoreProductFusion::deserialize(const std::string &path)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened())
    {
        throw FACELIB_EXCEPTION("can't read file " + path);
    }

    scoreNormalizer->deserialize(storage);

    learned = true;
}

// --- SVM Fusion ---

void ScoreSVMFusion::init()
{
    svm = cv::ml::SVM::create();
}

ScoreSVMFusion::ScoreSVMFusion()
{
    init();
}

void ScoreSVMFusion::serialize(const std::string &path) const
{
    svm->save(path);

    cv::FileStorage storage(path, cv::FileStorage::APPEND);
    scoreNormalizer->serialize(storage);
}

void ScoreSVMFusion::deserialize(const std::string &path)
{
    svm->load(path);

    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened())
    {
        throw FACELIB_EXCEPTION("can't read file " + path);
    }

    scoreNormalizer->deserialize(storage);

    learned = true;
}

void ScoreSVMFusion::learnImplementation()
{
    std::vector<Face::LinAlg::Vector> scores;
    std::vector<int> classes;
    prepareDataForClassification(scores, classes, 1, -1);

    cv::Mat data = colVectorsToFPMatrix(scores).t();
    cv::Mat labels = colVectorToColFPMatrix(classes);

    cv::Ptr<cv::ml::SVM> params = cv::ml::SVM::create();
    params->setType(cv::ml::SVM::C_SVC);
    params->setKernel(cv::ml::SVM::LINEAR);
    params->setTermCriteria(cv::TermCriteria(CV_TERMCRIT_ITER, 1000, 1e-6));
    params->train(data, cv::ml::ROW_SAMPLE, labels);
}

ScoreLevelFusionBase::Result ScoreSVMFusion::fuse(const std::vector<double> &scores) const
{
    Result result;
    result.preNormalized = scores;
    result.normalized = scoreNormalizer->normalize(scores);

    cv::Mat score = colVectorToColFPMatrix(result.normalized);
    result.score = svm->predict(score);
    return result;
}

cv::Mat ScoreSVMFusion::colVectorToColFPMatrix(std::vector<int> &vector) const
{
    int r = vector.size();
    cv::Mat result = cv::Mat::zeros(r, 1, CV_32F);
    for (int i = 0; i < r; i++)
    {
        float val = vector[i];
        result.at<float>(i) = val;
    }
    return result;
}

cv::Mat ScoreSVMFusion::colVectorToColFPMatrix(std::vector<double> &vector) const
{
    int r = vector.size();
    cv::Mat result = cv::Mat::zeros(r, 1, CV_32F);
    for (int i = 0; i < r; i++)
    {
        float val = vector[i];
        result.at<float>(i) = val;
    }
    return result;
}

cv::Mat ScoreSVMFusion::colVectorsToFPMatrix(std::vector<Face::LinAlg::Vector> &vectors) const
{
    int cols = vectors.size();
    int rows = vectors[0].rows;

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

// ---- GMM fusion ----

void ScoreGMMFusion::init()
{
    impostorScoresModel = cv::ml::EM::create();
    genuineScoresModel = cv::ml::EM::create();
}

ScoreGMMFusion::ScoreGMMFusion()
{
    init();
}

void ScoreGMMFusion::serialize(const std::string &path) const
{
    cv::FileStorage storageImp(path + "_imp", cv::FileStorage::WRITE);
    impostorScoresModel->write(storageImp);

    cv::FileStorage storageGen(path + "_gen", cv::FileStorage::WRITE);
    genuineScoresModel->write(storageGen);
}

void ScoreGMMFusion::deserialize(const std::string &path)
{
    cv::FileStorage storageImp(path + "_imp", cv::FileStorage::READ);
    if(!storageImp.isOpened())
    {
        throw FACELIB_EXCEPTION("can't read file " + path + "_imp");
    }
    impostorScoresModel->read(storageImp["StatModel.EM"]);

    cv::FileStorage storageGen(path + "_gen", cv::FileStorage::READ);
    if(!storageImp.isOpened())
    {
        throw FACELIB_EXCEPTION("can't read file " + path + "_gen");
    }
    genuineScoresModel->read(storageGen["StatModel.EM"]);

    learned = true;
}

void ScoreGMMFusion::learnImplementation()
{
    int n = components.size();
    Matrix genScores(components.front().genuineScores.size(), n);
    Matrix impScores(components.front().impostorScores.size(), n);
    for (int c = 0; c < n; c++)
    {
        const Evaluation &e = components[c];
        for (unsigned int r = 0; r < e.genuineScores.size(); r++)
        {
            double s = e.genuineScores[r];
            genScores(r, c) = s;
        }

        for (unsigned int r = 0; r < e.impostorScores.size(); r++)
        {
            double s = e.impostorScores[r];
            impScores(r, c) = s;
        }
    }

    impostorScoresModel->trainEM(impScores);
    genuineScoresModel->trainEM(genScores);
}

ScoreLevelFusionBase::Result ScoreGMMFusion::fuse(const std::vector<double> &scores) const
{
    Result result;
    result.preNormalized = scores;
    result.normalized = scoreNormalizer->normalize(scores);

    Matrix m = Face::LinAlg::Vector(scores).t();
    double i = impostorScoresModel->predict(m, cv::noArray(), 0);
    double g = genuineScoresModel->predict(m, cv::noArray(), 0);
    result.score = i - g;
    return result;
}
