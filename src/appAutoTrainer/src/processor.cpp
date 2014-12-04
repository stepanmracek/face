#include "processor.h"

#include <Poco/Path.h>
#include <Poco/File.h>

#include "common.h"
#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/facealigner.h"
#include "faceCommon/helpers/cmdlineargsparser.h"

using namespace Face::AutoTrainer;

Processor::Settings::Settings(int argc, char *argv[], bool &ok)
{
    Face::Helpers::CmdLineArgsParser cmdLineParser(argc, argv);

    meanFaceForAlign = cmdLineParser.getParamValue("--meanFaceForAlign", ok); if (!ok) return;
    preAlignTemplate = cmdLineParser.getParamValue("--preAlignTemplate", ok); if (!ok) return;
    inputDir = cmdLineParser.getParamValue("--inputDir", ok); if (!ok) return;
    outputDir = cmdLineParser.getParamValue("--outputDir", ok); if (!ok) return;

    ICPiters = cmdLineParser.getParamValueInt("--ICPiters", ok);
    if (!ok) ICPiters = 100;

    smoothCoef = cmdLineParser.getParamValueFloat("--smoothCoef", ok);
    if (!ok) smoothCoef = 0.01;

    smoothIters = cmdLineParser.getParamValueInt("--smoothIters", ok);
    if (!ok) smoothIters = 10;

    ok = true;
}

void Processor::Settings::printHelp()
{
    std::cout << "usage: tbs-face3d --process" << std::endl;
    std::cout << " mandatory parameters:" << std::endl;
    std::cout << "  --meanFaceForAlign model.obj" << std::endl;
    std::cout << "  --preAlignTemplate template.yml" << std::endl;
    std::cout << "  --inputDir /path/to/dir1/" << std::endl;
    std::cout << "  --outputDir /path/to/dir2/" << std::endl;
    std::cout << " optional parameters (with default values):" << std::endl;
    std::cout << "  --ICPiters 100" << std::endl;
    std::cout << "  --smoothCoef 0.01" << std::endl;
    std::cout << "  --smoothIters 10" << std::endl;
}

void Processor::Settings::printSettings()
{
    std::cout << "Settings:" << std::endl;
    std::cout << "  --meanFaceForAlign " << meanFaceForAlign << std::endl;
    std::cout << "  --preAlignTemplate " << preAlignTemplate << std::endl;
    std::cout << "  --inputDir " << inputDir << std::endl;
    std::cout << "  --outputDir " << outputDir << std::endl;
    std::cout << "  --ICPiters " << ICPiters << std::endl;
    std::cout << "  --smoothCoef " << smoothCoef << std::endl;
    std::cout << "  --smoothIters " << smoothIters << std::endl;
}

void Processor::process(const Settings &settings)
{
    Face::FaceData::FaceAligner aligner(
                Face::FaceData::Mesh::fromFile(settings.meanFaceForAlign), settings.preAlignTemplate);

    std::vector<int> ids;
    std::vector<Face::FaceData::Mesh> meshes;
    Common::loadMeshes(settings.inputDir, aligner, ids, meshes, settings.ICPiters,
                       settings.smoothIters, settings.smoothCoef, "-");

    // create directory if it doesn't exist
    Poco::File dir(settings.outputDir);
    if (!dir.exists())
    {
        std::cout << "creating path " << dir.path() << std::endl;
        dir.createDirectories();
    }

    int n = ids.size();
    for (int i = 0; i < n; i++)
    {
        meshes[i].writeBINZ(settings.outputDir + Poco::Path::separator() +
                            std::to_string(ids[i]) + "-" + std::to_string(i) + ".binz");
    }
}
