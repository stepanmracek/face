#include "evaluatethermo.h"

#include <Poco/Path.h>
#include <Poco/NumberParser.h>
#include <Poco/StringTokenizer.h>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/biometrics/multibiomertricsautotuner.h"

void EvaluateThermo::train()
{
    std::string pcaTrainPath = "/mnt/data/face/thermoNormalized/germany/p1";
    std::string fusionTrainPath = "/mnt/data/face/thermoNormalized/germany/p2";

    auto pcaTrainInput = Face::Biometrics::MultiBiomertricsAutoTuner::Input::fromDirectoryWithTextureImages(pcaTrainPath, "-");
    auto fusionTrainInput = Face::Biometrics::MultiBiomertricsAutoTuner::Input::fromDirectoryWithTextureImages(fusionTrainPath, "-");

    Face::Biometrics::MultiBiomertricsAutoTuner::Settings settings(
                Face::Biometrics::ScoreSVMFusion::name()+"-"+Face::Biometrics::ScoreNormalizerMean::name(),
                "../../test/units-thermo");
    auto extractor = Face::Biometrics::MultiBiomertricsAutoTuner::trainWithWrapper(pcaTrainInput, fusionTrainInput, settings);

    extractor->serialize("../../test/extractor-thermo");
}

void EvaluateThermo::evaluate()
{
    Face::Biometrics::MultiExtractor extractor("../../test/extractor-thermo");

    std::string basePath = "/mnt/data/face/thermoNormalized/";
    std::vector<std::string> dbPaths;
    dbPaths.push_back(basePath + "fitClean");
    dbPaths.push_back(basePath + "equinox");
    dbPaths.push_back(basePath + "notredame");

    for (const std::string &dbPath : dbPaths)
    {
        std::cout << dbPath << std::endl;
        std::vector<std::string> fileNames = Face::LinAlg::Loader::listFiles(dbPath, "*.png", Face::LinAlg::Loader::Filename);

        int n = fileNames.size();
        std::vector<Face::Biometrics::MultiTemplate> templates(n);

        std::cout << " reading images" << std::endl;
        #pragma omp parallel for
        for (int i = 0; i < n; i++)
        {
            int id = Poco::NumberParser::parse(Poco::StringTokenizer(fileNames[i], "-")[0]);
            ImageGrayscale img = cv::imread(dbPath + Poco::Path::separator() + fileNames[i], cv::IMREAD_GRAYSCALE);
            Face::Biometrics::MultiExtractor::ImageData imgData(img);
            templates[i] = extractor.extract(imgData, 1, id);
        }

        std::cout << " Evaluating" << std::endl;
        auto eval = extractor.evaluate(templates);
        eval.printStats();
    }
}
