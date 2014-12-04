#include "trainer.h"

#include "faceCommon/facedata/facealigner.h"
#include "faceCommon/linalg/loader.h"
#include "faceCommon/facedata/mdenoise/mdenoise.h"
#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceCommon/helpers/cmdlineargsparser.h"
#include "common.h"

using namespace Face::AutoTrainer;

Trainer::Settings::Settings(int argc, char *argv[], bool &ok)
{
    Face::Helpers::CmdLineArgsParser cmdLineParser(argc, argv);

    meanFaceForAlign = cmdLineParser.getParamValue("--meanFaceForAlign", ok); if (!ok) return;
    preAlignTemplate = cmdLineParser.getParamValue("--preAlignTemplate", ok); if (!ok) return;
    unitsFile = cmdLineParser.getParamValue("--unitsFile", ok); if (!ok) return;
    fusionName = cmdLineParser.getParamValue("--fusionName", ok); if (!ok) return;
    frgcDir = cmdLineParser.getParamValue("--frgcDir", ok); if (!ok) return;
    trainDir = cmdLineParser.getParamValue("--trainDir", ok); if (!ok) return;

    frgcSamples = cmdLineParser.getParamValueInt("--frgcSamples", ok);
    if (!ok) frgcSamples = 300;

    ICPiters = cmdLineParser.getParamValueInt("--ICPiters", ok);
    if (!ok) ICPiters = 50;

    smoothCoef = cmdLineParser.getParamValueFloat("--smoothCoef", ok);
    if (!ok) smoothCoef = 0.01;

    smoothIters = cmdLineParser.getParamValueFloat("--smoothIters", ok);
    if (!ok) smoothIters = 10;

    ok = true;
}

void Trainer::Settings::printHelp()
{
    std::cout << "usage: appAutoTrainer --trainUnits outputDir" << std::endl;
    std::cout << " mandatory parameters:" << std::endl;
    std::cout << "  --meanFaceForAlign model.obj" << std::endl;
    std::cout << "  --preAlignTemplate template.yml" << std::endl;
    std::cout << "  --unitsFile units.txt" << std::endl;
    std::cout << "  --fusionName [" << Face::Biometrics::ScoreSVMFusion::name() << "-mean|"
        << Face::Biometrics::ScoreWeightedSumFusion::name() << "-mean|...]" << std::endl;
    std::cout << "  --frgcDir /path/to/exported/aligned/frgc/meshes/" << std::endl;
    std::cout << "  --trainDir /path/to/dir/containing/training/unaligned/meshes/" << std::endl;
    std::cout << " optional parameters (with default values):" << std::endl;
    std::cout << "  --frgcSamples 300" << std::endl;
    std::cout << "  --ICPiters 50" << std::endl;
    std::cout << "  --smoothCoef 0.01" << std::endl;
    std::cout << "  --smoothIters 10" << std::endl;
}

void Trainer::Settings::printSettings()
{
    std::cout << "Settings:" << std::endl;
    std::cout << "  --meanFaceForAlign " << meanFaceForAlign << std::endl;
    std::cout << "  --preAlignTemplate " << preAlignTemplate << std::endl;
    std::cout << "  --unitsFile " << unitsFile << std::endl;
    std::cout << "  --fusionName " << fusionName << std::endl;
    std::cout << "  --frgcDir " << frgcDir << std::endl;
    std::cout << "  --trainDir " << trainDir << std::endl;
    std::cout << "  --frgcSamples " << frgcSamples << std::endl;
    std::cout << "  --ICPiters " << ICPiters << std::endl;
    std::cout << "  --smoothCoef " << smoothCoef << std::endl;
    std::cout << "  --smoothIters " << smoothIters << std::endl;
}

Face::Biometrics::MultiExtractor::Ptr Trainer::train(const Settings &settings)
{
    Face::FaceData::FaceAligner aligner(
                Face::FaceData::Mesh::fromFile(settings.meanFaceForAlign), settings.preAlignTemplate);

    std::cout << "pca training data" << std::endl;
    std::vector<int> ids;
    std::vector<Face::FaceData::Mesh> meshes;
    Common::loadMeshes(settings.trainDir, aligner, ids, meshes,
                       settings.ICPiters, settings.smoothIters, settings.smoothCoef, "-");
    Face::Biometrics::MultiBiomertricsAutoTuner::Input trainData =
            Face::Biometrics::MultiBiomertricsAutoTuner::Input::fromAlignedMeshes(ids, meshes);
    std::cout << "loaded " << trainData.ids.size() << " scans" << std::endl;

    std::cout << "fusion training data" << std::endl;
    Face::Biometrics::MultiBiomertricsAutoTuner::Input frgcData =
            Face::Biometrics::MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(
                settings.frgcDir, "d", settings.frgcSamples);
    std::cout << "loaded " << frgcData.ids.size() << " scans" << std::endl;

    Face::Biometrics::MultiBiomertricsAutoTuner::Settings autoTunerSettings(settings.fusionName, settings.unitsFile);
    Face::Biometrics::MultiExtractor::Ptr extractor = Face::Biometrics::MultiBiomertricsAutoTuner::trainWithWrapper(
                frgcData, trainData, autoTunerSettings);

    return extractor;
}
