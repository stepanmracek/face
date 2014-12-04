#include "templateextractor.h"

#include <Poco/File.h>
#include <Poco/Path.h>

#include "common.h"
#include "faceCommon/helpers/cmdlineargsparser.h"
#include "faceCommon/biometrics/multiextractor.h"
#include "faceCommon/facedata/facealigner.h"

using namespace Face::AutoTrainer;

TemplateExtractor::Settings::Settings(int argc, char *argv[], bool &ok)
{
    Face::Helpers::CmdLineArgsParser cmdLineParser(argc, argv);

    extractorPath = cmdLineParser.getParamValue("--extractor", ok); if (!ok) return;
    resultPath = cmdLineParser.getParamValue("--resultPath", ok); if (!ok) return;
    inputPath = cmdLineParser.getParamValue("--inputPath", ok); if (!ok) return;
    meanFaceForAlign = cmdLineParser.getParamValue("--meanFaceForAlign", ok); if (!ok) return;
    preAlignTemplate = cmdLineParser.getParamValue("--preAlignTemplate", ok); if (!ok) return;

    ICPiters = cmdLineParser.getParamValueInt("--ICPiters", ok);
    if (!ok) ICPiters = 100;

    smoothCoef = cmdLineParser.getParamValueFloat("--smoothCoef", ok);
    if (!ok) smoothCoef = 0.01;

    smoothIters = cmdLineParser.getParamValueInt("--smoothIters", ok);
    if (!ok) smoothIters = 10;

    templateVersion = cmdLineParser.getParamValueInt("--templateVersion", ok);
    if (!ok) templateVersion = 1;

    ok = true;
}

void TemplateExtractor::Settings::printHelp()
{
    std::cout << "usage: tbs-face3d --extract" << std::endl;
    std::cout << " mandatory parameters:" << std::endl;
    std::cout << "  --extractor path/to/extractor" << std::endl;
    std::cout << "  --inputPath path/to/input/meshes" << std::endl;
    std::cout << "  --resultPath path/to/resulting/templates" << std::endl;
    std::cout << "  --meanFaceForAlign path/to/mean/face.obj" << std::endl;
    std::cout << "  --preAlignTemplate path/to/mean/template.yml" << std::endl;
    std::cout << " optional parameters (with default values):" << std::endl;
    std::cout << "  --ICPiters 100" << std::endl;
    std::cout << "  --smoothCoef 0.01" << std::endl;
    std::cout << "  --smoothIters 10" << std::endl;
    std::cout << "  --templateVersion 1" << std::endl;
}

void TemplateExtractor::Settings::printSettings()
{
    std::cout << "Settings:" << std::endl;
    std::cout << "  --extractor " << extractorPath << std::endl;
    std::cout << "  --inputPath " << inputPath << std::endl;
    std::cout << "  --resultPath " << resultPath << std::endl;
    std::cout << "  --meanFaceForAlign " << meanFaceForAlign << std::endl;
    std::cout << "  --preAlignTemplate " << preAlignTemplate << std::endl;
    std::cout << "  --ICPiters " << ICPiters << std::endl;
    std::cout << "  --smoothCoef " << smoothCoef << std::endl;
    std::cout << "  --smoothIters " << smoothIters << std::endl;
    std::cout << "  --templateVersion " << templateVersion << std::endl;
}

void TemplateExtractor::extract(const Settings &settings)
{
    Face::Biometrics::MultiExtractor extractor(settings.extractorPath);
    Face::FaceData::FaceAligner aligner(
                Face::FaceData::Mesh::fromFile(settings.meanFaceForAlign), settings.preAlignTemplate);

    std::vector<int> ids;
    std::vector<Face::FaceData::Mesh> meshes;
    Common::loadMeshes(settings.inputPath, aligner, ids, meshes, settings.ICPiters,
                       settings.smoothIters, settings.smoothCoef,  "-");

    // create directory if it doesn't exist
    Poco::File dir(settings.resultPath);
    if (!dir.exists())
    {
        std::cout << "creating path " << dir.path() << std::endl;
        dir.createDirectories();
    }

    int n = ids.size();
    #pragma omp parallel for
    for (int i = 0; i < n; i++)
    {
        Face::Biometrics::MultiTemplate t = extractor.extract(meshes[i], settings.templateVersion, ids[i]);
        std::string path = settings.resultPath + Poco::Path::separator() + std::to_string(ids[i]) + "-" + std::to_string(i) + ".gz";
        cv::FileStorage storage(path, cv::FileStorage::WRITE);
        t.serialize(storage);
    }
}
