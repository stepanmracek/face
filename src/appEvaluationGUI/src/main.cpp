#include <QApplication>
#include <Poco/StringTokenizer.h>

#include "faceExtras/gui/widgetevaluation.h"
#include "faceCommon/helpers/cmdlineargsparser.h"
#include "faceCommon/biometrics/multiextractor.h"
#include "faceCommon/facedata/facealigner.h"
#include "faceCommon/linalg/loader.h"

int printHelpAndExit(int code)
{
    std::cout << "TODO: help message" << std::endl;
    exit(code);
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Face::GUI::WidgetEvaluation widget;

    /*bool ok;

    Face::Helpers::CmdLineArgsParser cmdLineParser(argc, argv);
    std::string meanFaceForAlign = cmdLineParser.getParamValue("--meanFaceForAlign", ok); if (!ok) printHelpAndExit(1);
    std::string preAlignTemplate = cmdLineParser.getParamValue("--preAlignTemplate", ok); if (!ok) printHelpAndExit(1);
    std::string extractorsPath = cmdLineParser.getParamValue("--extractors", ok); if (!ok) printHelpAndExit(1);
    std::string dataDir = cmdLineParser.getParamValue("--dataDir", ok); if (!ok) printHelpAndExit(1);

    int ICPiters = cmdLineParser.getParamValueInt("--ICPiters", ok);
    if (!ok) ICPiters = 100;

    double smoothCoef = cmdLineParser.getParamValueFloat("--smoothCoef", ok);
    if (!ok) smoothCoef = 0.01;

    int smoothIters = cmdLineParser.getParamValueInt("--smoothIters", ok);
    if (!ok) smoothIters = 10;

    Face::FaceData::FaceAligner aligner(Face::FaceData::Mesh::fromFile(meanFaceForAlign), preAlignTemplate);
    std::vector<int> ids;
    std::vector<Face::FaceData::Mesh> meshes;

    std::cout << "Loading meshes..." << std::endl;
    Face::LinAlg::Loader::loadMeshes(dataDir, aligner, ids, meshes, ICPiters, smoothIters, smoothCoef, "-");

    Poco::StringTokenizer tokenizer(extractorsPath, ";", (Poco::StringTokenizer::TOK_IGNORE_EMPTY | Poco::StringTokenizer::TOK_TRIM));
    for (const std::string extractorPath : tokenizer)
    {
        std::cout << extractorPath << std::endl;
        Face::Biometrics::MultiExtractor extractor(extractorPath);
        auto templates = extractor.extract(meshes, ids, 0);
        widget.addEvaluation(QString::fromStdString(extractorPath), extractor.evaluate(templates));
    }*/

    widget.show();
    return app.exec();
}
