#include "faceCommon/biometrics/multibiomertricsautotuner.h"

#include <Poco/Path.h>
#include <Poco/NumberParser.h>
#include <Poco/StringTokenizer.h>
#include <Poco/Timestamp.h>
#include <fstream>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/biometrics/scorelevelfusionwrapper.h"
#include "faceCommon/biometrics/template.h"

using namespace Face::Biometrics;

MultiBiomertricsAutoTuner::Input::Input()
{

}

MultiBiomertricsAutoTuner::Input MultiBiomertricsAutoTuner::Input::fromAlignedMeshes(
        const std::vector<int> &ids, const std::vector<Face::FaceData::Mesh> meshes)
{
    Input result;
    result.ids = ids;
    for (const FaceData::Mesh &mesh : meshes)
    {
        result.imageData.push_back(MultiExtractor::ImageData(mesh));
    }
    return result;
}

MultiBiomertricsAutoTuner::Input MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(const std::string &path,
                                                                                                  const std::string &idAndScanSeparator,
                                                                                                  int maxCount, int startIndex)
{
    std::string dir = path;
    if (dir.back() != Poco::Path::separator()) dir.push_back(Poco::Path::separator());

    Input result;
    std::vector<std::string> nameFilters;
    nameFilters.push_back("*.bin"); nameFilters.push_back("*.binz"); nameFilters.push_back("*.obj");
    std::vector<std::string> files = Face::LinAlg::Loader::listFiles(dir, nameFilters, Face::LinAlg::Loader::Filename);

    if (maxCount == -1)
    {
        maxCount = files.size() - startIndex;
    }

    // Allocate memory
    result.ids.resize(maxCount);
    result.imageData.resize(maxCount);

    #pragma omp parallel for
    for (int i = startIndex; i < startIndex+maxCount; i++)
    {
        const std::string &f = files[i];
        result.ids[i-startIndex] = Poco::NumberParser::parse(Poco::StringTokenizer(f, idAndScanSeparator)[0]);

        FaceData::Mesh mesh = FaceData::Mesh::fromFile(dir + f);
        result.imageData[i - startIndex] = MultiExtractor::ImageData(mesh);
    }

    return result;
}

MultiBiomertricsAutoTuner::Input MultiBiomertricsAutoTuner::Input::fromDirectoryWithTextureImages(const std::string &path,
                                                                                                  const std::string &idAndScanSeparator,
                                                                                                  int maxCount, int startIndex)
{
    std::string dir = path;
    if (dir.back() != Poco::Path::separator()) dir.push_back(Poco::Path::separator());

    Input result;
    std::vector<std::string> files = Face::LinAlg::Loader::listFiles(dir, "*.png", Face::LinAlg::Loader::Filename);

    if (maxCount == -1)
    {
        maxCount = files.size() - startIndex;
    }

    // Allocate memory
    result.ids.resize(maxCount);
    result.imageData.resize(maxCount);

    #pragma omp parallel for
    for (int i = startIndex; i < startIndex+maxCount; i++)
    {
        const std::string &f = files[i];
        result.ids[i-startIndex] = Poco::NumberParser::parse(Poco::StringTokenizer(f, idAndScanSeparator)[0]);

        ImageGrayscale img = cv::imread((dir + f), cv::IMREAD_GRAYSCALE);
        result.imageData[i - startIndex] = MultiExtractor::ImageData(img);
    }

    return result;
}

MultiBiomertricsAutoTuner::Settings::Settings() :
    fusionType(ScoreWeightedSumFusion::name())
{

}

MultiBiomertricsAutoTuner::Settings::Settings(const std::string &fusionType, const std::string &unitsParametersFile) :
    fusionType(fusionType)
{
    std::ifstream in(unitsParametersFile);
    std::string line;

    while (std::getline(in, line))
    {
        if (line.empty()) continue;
        params.push_back(line);
    }
}

void MultiBiomertricsAutoTuner::fillEvaluations(const Input &sourceDatabaseTrainData,
                                                const Input &targetDatabaseTrainData,
                                                const Settings &settings, std::vector<Evaluation> &evaluations)
{
    int allUnitsCount = settings.params.size();
    Poco::Timestamp stamp;
    #pragma omp parallel for
    for (int i = 0; i < allUnitsCount; i++)
    {
        trainUnit(settings.params[i], sourceDatabaseTrainData, targetDatabaseTrainData, evaluations, i);
    }
    std::cout << "Training individual classifiers took " << (stamp.elapsed()/1000) << "ms";
}

MultiExtractor::Ptr MultiBiomertricsAutoTuner::trainWithWrapper(const Input &sourceDatabaseTrainData,
                                                                const Input &targetDatabaseTrainData,
                                                                const Settings &settings)
{
    int allUnitsCount = settings.params.size();
    std::vector<Evaluation> evaluations(allUnitsCount);
    fillEvaluations(sourceDatabaseTrainData, targetDatabaseTrainData, settings, evaluations);

    MultiExtractor::Ptr extractor = new MultiExtractor();

    ScoreLevelFusionWrapper::Result wrapperResult
            = ScoreLevelFusionWrapper::trainClassifier(settings.fusionType, evaluations, true);

    extractor->fusion = wrapperResult.fusion;
    for (int selectedUnitIndex : wrapperResult.selectedComponents)
    {
        extractor->units.push_back(trainUnit(settings.params[selectedUnitIndex], sourceDatabaseTrainData,
                                             targetDatabaseTrainData, evaluations, selectedUnitIndex));
    }

    return extractor;
}

MultiExtractor::Ptr MultiBiomertricsAutoTuner::trainAllUnits(const Input &sourceDatabaseTrainData,
                                                             const Input &targetDatabaseTrainData,
                                                             const Settings &settings)
{
    int allUnitsCount = settings.params.size();
    MultiExtractor::Ptr extractor = new MultiExtractor();
    extractor->fusion = ScoreLevelFusionFactory::create(settings.fusionType);
    std::vector<Evaluation> evaluations(allUnitsCount);
    for (int i = 0; i < allUnitsCount; i++)
    {
        extractor->units.push_back(trainUnit(settings.params[i], sourceDatabaseTrainData, targetDatabaseTrainData, evaluations, i));
        extractor->fusion->addComponent(evaluations[i]);
    }
    extractor->fusion->learn();
    return extractor;
}

MultiExtractor::Unit::Ptr MultiBiomertricsAutoTuner::trainUnit(const std::string &lineParams,
                                                               const Input &sourceDatabaseTrainData,
                                                               const Input &targetDatabaseTrainData,
                                                               std::vector<Evaluation> &evaluations, int index)
{
    MultiExtractor::Unit::Ptr unit = MultiExtractor::Unit::parse(lineParams);
    unit->train(sourceDatabaseTrainData.ids, sourceDatabaseTrainData.imageData);
    std::vector<Template> templates;
    int n = targetDatabaseTrainData.ids.size();
    for (int i = 0; i < n; i++)
    {
        templates.push_back(Template(targetDatabaseTrainData.ids[i], unit->extract(targetDatabaseTrainData.imageData[i])));
    }
    Evaluation eval(templates, *(unit->metrics));
    evaluations[index] = eval;
    std::cout << "  " << index << " " << unit->writeParams() << ": " << eval.eer;

    return unit;
}
