#include "evaluator.h"

#include "common.h"
#include "faceCommon/helpers/cmdlineargsparser.h"
#include "faceCommon/biometrics/multiextractor.h"

using namespace Face::AutoTrainer;

Evaluator::Settings::Settings(int argc, char *argv[], bool &ok)
{
    Face::Helpers::CmdLineArgsParser cmdLineParser(argc, argv);

    meanFaceForAlign = cmdLineParser.getParamValue("--meanFaceForAlign", ok); if (!ok) return;
    preAlignTemplate = cmdLineParser.getParamValue("--preAlignTemplate", ok); if (!ok) return;
    extractor = cmdLineParser.getParamValue("--extractor", ok); if (!ok) return;
    evalDir = cmdLineParser.getParamValue("--evalDir", ok); if (!ok) return;

    frgcSamples = cmdLineParser.getParamValueInt("--frgcSamples", ok);
    if (!ok) frgcSamples = 200;

    ICPiters = cmdLineParser.getParamValueInt("--ICPiters", ok);
    if (!ok) ICPiters = 100;

    smoothCoef = cmdLineParser.getParamValueFloat("--smoothCoef", ok);
    if (!ok) smoothCoef = 0.01;

    smoothIters = cmdLineParser.getParamValueInt("--smoothIters", ok);
    if (!ok) smoothIters = 10;

    ok = true;
}

void Evaluator::Settings::printHelp()
{
    std::cout << "usage: appAutoTrainer --evaluate output" << std::endl;
    std::cout << " mandatory parameters:" << std::endl;
    std::cout << "  --extractor /path/to/extractor/" << std::endl;
    std::cout << "  --meanFaceForAlign model.obj" << std::endl;
    std::cout << "  --preAlignTemplate template.yml" << std::endl;
    std::cout << "  --evalDir /path/to/dir/containing/evaluation/unaligned/meshes/" << std::endl;
    std::cout << " optional parameters (with default values):" << std::endl;
    std::cout << "  --ICPiters 100" << std::endl;
    std::cout << "  --smoothCoef 0.01" << std::endl;
    std::cout << "  --smoothIters 10" << std::endl;
}

void Evaluator::Settings::printSettings()
{
    std::cout << "Settings:" << std::endl;
    std::cout << "  --extractor " << extractor << std::endl;
    std::cout << "  --meanFaceForAlign " << meanFaceForAlign << std::endl;
    std::cout << "  --preAlignTemplate " << preAlignTemplate << std::endl;
    std::cout << "  --evalDir " << evalDir << std::endl;
    std::cout << "  --ICPiters " << ICPiters << std::endl;
    std::cout << "  --smoothCoef " << smoothCoef << std::endl;
    std::cout << "  --smoothIters " << smoothIters << std::endl;
}

Face::Biometrics::Evaluation Evaluator::evaluate(const Settings &settings)
{
    Face::FaceData::FaceAligner aligner(
                Face::FaceData::Mesh::fromFile(settings.meanFaceForAlign), settings.preAlignTemplate);

    std::cout << "evaluation data" << std::endl;
    std::vector<int> ids;
    std::vector<Face::FaceData::Mesh> meshes;
    Common::loadMeshes(settings.evalDir, aligner, ids, meshes,
                       settings.ICPiters, settings.smoothIters, settings.smoothCoef, "-");

    std::cout << "extractor" << std::endl;
    Face::Biometrics::MultiExtractor extractor(settings.extractor);

    std::cout << "extracting templates" << std::endl;
    std::vector<Face::Biometrics::MultiTemplate> templates;
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        templates.push_back(extractor.extract(meshes[i], 0, ids[i]));
    }

    std::cout << "evaluating" << std::endl;
    return extractor.evaluate(templates);
}
