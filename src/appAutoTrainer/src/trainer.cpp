#include "trainer.h"

#include "faceCommon/facedata/facealigner.h"
#include "faceCommon/linalg/loader.h"
#include "faceCommon/facedata/mdenoise/mdenoise.h"
#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceCommon/helpers/cmdlineargsparser.h"

using namespace Face::AutoTrainer;

Trainer::Settings::Settings(int argc, char *argv[], bool &ok)
{
    Face::Settings &s = Face::Settings::instance();

    Face::Helpers::CmdLineArgsParser cmdLineParser(argc, argv);    

    unitsFile = cmdLineParser.getParamValue("--unitsFile", ok); if (!ok) return;
    fusionName = cmdLineParser.getParamValue("--fusionName", ok); if (!ok) return;
    frgcDir = cmdLineParser.getParamValue("--frgcDir", ok); if (!ok) return;
    trainDir = cmdLineParser.getParamValue("--trainDir", ok); if (!ok) return;

    frgcSamples = cmdLineParser.getParamValueInt("--frgcSamples", ok);
    if (!ok) frgcSamples = 300;

    noWrapper = cmdLineParser.hasParam("--noWrapper");

    if (!parseAlignType(cmdLineParser))
    {
        ok = false;
        return;
    }

    parseSettings(cmdLineParser);

    ok = true;
}

void Trainer::Settings::printHelp()
{
    std::cout << "usage: tbs-face3d --trainUnits outputDir" << std::endl;
    std::cout << " mandatory parameters:" << std::endl;
    std::cout << "  --unitsFile units.txt" << std::endl;
    std::cout << "  --fusionName [" << Face::Biometrics::ScoreSVMFusion::name() << "-mean|"
        << Face::Biometrics::ScoreWeightedSumFusion::name() << "-mean|...]" << std::endl;
    std::cout << "  --frgcDir /path/to/exported/aligned/frgc/meshes/" << std::endl;
    std::cout << "  --trainDir /path/to/dir/containing/training/unaligned/meshes/" << std::endl;
    std::cout << "  --align [none|icp|landmark]" << std::endl;
    std::cout << " optional parameters (with default values):" << std::endl;
    SettingsBase::printHelp();
    std::cout << "  --noWrapper" << std::endl;
}

void Trainer::Settings::printSettings()
{
    Face::Settings &s = Face::Settings::instance();

    std::cout << "Settings:" << std::endl;
    std::cout << "  --unitsFile " << unitsFile << std::endl;
    std::cout << "  --fusionName " << fusionName << std::endl;
    std::cout << "  --frgcDir " << frgcDir << std::endl;
    std::cout << "  --trainDir " << trainDir << std::endl;
    std::cout << "  --frgcSamples " << frgcSamples << std::endl;
    SettingsBase::printSettings();
    std::cout << "  --noWrapper " << noWrapper << std::endl;
}

Face::Biometrics::MultiExtractor::Ptr Trainer::train(const Settings &settings)
{
    std::cout << "FRGC training data" << std::endl;
    Face::Biometrics::MultiBiomertricsAutoTuner::Input frgcData =
            Face::Biometrics::MultiBiomertricsAutoTuner::Input::fromDirectoryWithAlignedMeshes(
                settings.frgcDir, "d", settings.frgcSamples);
    std::cout << "loaded " << frgcData.ids.size() << " scans" << std::endl;

    std::cout << "fusion training data" << std::endl;
    std::vector<int> ids;
    std::vector<Face::FaceData::Mesh> meshes;

    if (settings.alignType == Settings::AlignType::ICP)
    {
        Face::FaceData::FaceAlignerIcp aligner;
        Face::LinAlg::Loader::loadMeshes(settings.trainDir, aligner, ids, meshes,
                                         settings.ICPiters, settings.smoothIters, settings.smoothCoef, "-");
    }
    else if (settings.alignType == Settings::AlignType::Landmark)
    {
        Face::FaceData::FaceAlignerLandmark aligner;
        Face::LinAlg::Loader::loadMeshes(settings.trainDir, aligner, ids, meshes,
                                         settings.smoothIters, settings.smoothCoef, "-");
    } else {
        Face::LinAlg::Loader::loadMeshes(settings.trainDir, ids, meshes,
                                         settings.smoothIters, settings.smoothCoef, "-");
    }

    Face::Biometrics::MultiBiomertricsAutoTuner::Input trainData =
            Face::Biometrics::MultiBiomertricsAutoTuner::Input::fromAlignedMeshes(ids, meshes);
    std::cout << "loaded " << trainData.ids.size() << " scans" << std::endl;

    Face::Biometrics::MultiBiomertricsAutoTuner::Settings autoTunerSettings(settings.fusionName, settings.unitsFile);
    Face::Biometrics::MultiExtractor::Ptr extractor = settings.noWrapper ?
                Face::Biometrics::MultiBiomertricsAutoTuner::trainAllUnits(frgcData, trainData, autoTunerSettings) :
                Face::Biometrics::MultiBiomertricsAutoTuner::trainWithWrapper(frgcData, trainData, autoTunerSettings);

    return extractor;
}
