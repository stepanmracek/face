#include "evaluatemultiextractor.h"

#include "faceCommon/biometrics/multibiomertricsautotuner.h"
#include "faceCommon/helpers/frgcutils.h"
#include "faceCommon/linalg/loader.h"

#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>

using namespace Face::Biometrics;
using namespace Face::LinAlg;
using namespace Face::FaceData;
using namespace Face::Helpers;

void EvaluateMultiExtractor::createFRGCextractor()
{
    std::string unitsPath("../../test/units-master");
    std::string frgcPath("/home/stepo/data/frgc/spring2004/zbin-aligned2");
    MultiBiomertricsAutoTuner::Settings settings(ScoreSVMFusion::name() + "-" + ScoreNormalizerMean::name(), unitsPath);
    auto trainData = MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(frgcPath, "d", 416, 0);
    auto testData = MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(frgcPath, "d", 451, 416);
    auto extractor = MultiBiomertricsAutoTuner::trainWithWrapper(trainData, testData, settings);
    extractor->serialize("frgcExtractor-master");
}

void EvaluateMultiExtractor::evaluateFRGCextractor()
{
    std::string frgcPath("/home/stepo/data/frgc/spring2004/zbin-aligned2");
    std::vector<std::string> files = Face::LinAlg::Loader::listFiles(frgcPath, "*.binz", Face::LinAlg::Loader::Filename);
    MultiExtractor extractor("../../test/frgcExtractor-master");

    for (int part = 3; part <= 5; part++)
    {
        std::vector<MultiTemplate> testTemplates(FRGCUtils::Spring2004::partSize(part));

        int n = FRGCUtils::Spring2004::partSize(part);
        int start = FRGCUtils::Spring2004::partStart(part);

        #pragma omp parallel for
        for (int i = 0; i < n; i++)
        {
            int id = Poco::NumberParser::parse(Poco::StringTokenizer(files[i + start], "d")[0]);
            auto mesh = Mesh::fromFile(frgcPath + files[i + start]);
            testTemplates[i] = extractor.extract(mesh, 1, id);
        }
        auto evaluationResult = extractor.evaluate(testTemplates);
        evaluationResult.printStats();
    }
}

void EvaluateMultiExtractor::evaluateFRGCUnits()
{
    std::string unitsPath("../../test/units-master");
    std::string frgcPath("/home/stepo/data/frgc/spring2004/zbin-aligned2");
    MultiBiomertricsAutoTuner::Settings settings(std::string(), unitsPath);
    std::cout << "loading pca train data" << std::endl;
    auto trainData = MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(frgcPath, "d", 416, 0);
    std::cout << "loading evaluation data" << std::endl;
    auto testData = MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(frgcPath, "d", 451, 416);
    int n = settings.params.size();
    std::vector<Evaluation> evals(n);
    #pragma omp parallel for
    for (int i = 0; i < n; i++)
    {
        MultiBiomertricsAutoTuner::trainUnit(settings.params[i], trainData, testData, evals, i);
    }
}

void EvaluateMultiExtractor::compareExtractors()
{
    std::string unitsPath("../../test/units-master-frgc-trained");
    std::string frgcPath("/home/stepo/data/frgc/spring2004/zbin-aligned2");
    std::vector<std::string> fusionNames;
    fusionNames.push_back(ScoreSumFusion::name());
    fusionNames.push_back(ScoreLDAFusion::name());
    fusionNames.push_back(ScoreGMMFusion::name());
    fusionNames.push_back(ScoreLogisticRegressionFusion::name());
    fusionNames.push_back(ScoreWeightedSumFusion::name());
    fusionNames.push_back(ScoreSVMFusion::name());

    auto trainPCAData = MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes
            (frgcPath, "d",FRGCUtils::Spring2004::part1Size(), FRGCUtils::Spring2004::part1Start());

    auto trainFusionData = MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes
            (frgcPath, "d", FRGCUtils::Spring2004::part2Size(), FRGCUtils::Spring2004::part2Start());

    int trainCount = trainFusionData.ids.size();

    auto evalData = MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes
            (frgcPath, "d", FRGCUtils::Spring2004::part3Size(), FRGCUtils::Spring2004::part3Start());

    int evalCount = evalData.ids.size();

    for (const std::string &fusionName : fusionNames)
    {
        std::cout << fusionName << std::endl;
        MultiBiomertricsAutoTuner::Settings settings(fusionName + "-" + ScoreNormalizerMean::name(), unitsPath);
        auto extractor = MultiBiomertricsAutoTuner::trainAllUnits(trainPCAData, trainFusionData, settings);

        std::vector<MultiTemplate> trainTemplates(trainCount);
        #pragma omp parallel for
        for (int i = 0; i < trainCount; i++)
        {
            trainTemplates[i] = extractor->extract(trainFusionData.imageData[i], 1, trainFusionData.ids[i]);
        }
        auto eval = extractor->evaluate(trainTemplates);
        eval.printStats();

        std::vector<MultiTemplate> evalTemplates(evalCount);
        #pragma omp parallel for
        for (int i = 0; i < evalCount; i++)
        {
            evalTemplates[i] = extractor->extract(evalData.imageData[i], 1, evalData.ids[i]);
        }

        eval = extractor->evaluate(evalTemplates);
        eval.printStats();
    }
}
