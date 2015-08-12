#include "evaluator.h"

#include "faceCommon/helpers/cmdlineargsparser.h"
#include "faceCommon/biometrics/multiextractor.h"
#include "faceCommon/facedata/facealigner.h"
#include "faceCommon/linalg/loader.h"

using namespace Face::AutoTrainer;

Evaluator::Settings::Settings(int argc, char *argv[], bool &ok)
{
    Face::Settings &s = Face::Settings::instance();
    Face::Helpers::CmdLineArgsParser cmdLineParser(argc, argv);

    extractor = cmdLineParser.getParamValue("--extractor", ok); if (!ok) return;
    evalDir = cmdLineParser.getParamValue("--evalDir", ok); if (!ok) return;

    if (!parseAlignType(cmdLineParser))
    {
        ok = false;
        return;
    }

    parseSettings(cmdLineParser);

    ok = true;
}

void Evaluator::Settings::printHelp()
{
    std::cout << "usage: tbs-face3d --evaluate output" << std::endl;
    std::cout << " mandatory parameters:" << std::endl;
    std::cout << "  --extractor /path/to/extractor/" << std::endl;
    std::cout << "  --evalDir /path/to/dir/containing/evaluation/unaligned/meshes/" << std::endl;
    std::cout << "  --align [none|icp|landmark]" << std::endl;
    std::cout << " optional parameters (with default values):" << std::endl;
    SettingsBase::printHelp();
}

void Evaluator::Settings::printSettings()
{
    Face::Settings &s = Face::Settings::instance();

    std::cout << "Settings:" << std::endl;
    std::cout << "  --extractor " << extractor << std::endl;  
    std::cout << "  --evalDir " << evalDir << std::endl;
    SettingsBase::printSettings();
}

Face::Biometrics::Evaluation Evaluator::evaluate(const Settings &settings)
{
    std::cout << "evaluation data" << std::endl;
    std::vector<int> ids;
    std::vector<Face::FaceData::Mesh> meshes;

    if (settings.alignType == Settings::AlignType::ICP)
    {
        Face::FaceData::FaceAlignerIcp aligner;
        Face::LinAlg::Loader::loadMeshes(settings.evalDir, aligner, ids, meshes,
                                         settings.ICPiters, settings.smoothIters, settings.smoothCoef, "-");
    }
    else if (settings.alignType == Settings::AlignType::Landmark)
    {
        Face::FaceData::FaceAlignerLandmark aligner;
        Face::LinAlg::Loader::loadMeshes(settings.evalDir, aligner, ids, meshes,
                                         settings.smoothIters, settings.smoothCoef, "-");
    } else {
        Face::LinAlg::Loader::loadMeshes(settings.evalDir, ids, meshes,
                                         settings.smoothIters, settings.smoothCoef, "-");
    }

    std::cout << "extractor" << std::endl;
    Face::Biometrics::MultiExtractor extractor(settings.extractor);

    std::cout << "extracting templates" << std::endl;
    std::vector<Face::Biometrics::MultiTemplate> templates = extractor.extract(meshes, ids, 0);

    std::cout << "evaluating" << std::endl;
    return extractor.evaluate(templates);
}
