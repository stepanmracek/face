#include <Poco/Timestamp.h>

#include "faceCommon/helpers/cmdlineargsparser.h"
#include "faceCommon/facedata/facealigner.h"
#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceCommon/biometrics/multiextractor.h"

void printHelpAndExit(char *appName)
{
    std::cout << "usage: " << appName << std::endl;
    std::cout << "  --extractor path/to/extractor" << std::endl;
    std::cout << "  --inputMesh path/to/test/input/mesh" << std::endl;
    std::cout << "  --landmarks path/to/landmarks" << std::endl;
    std::cout << "  --landmarksModel path/to/landmarksModel" << std::endl;
    std::cout << "  [--threadPool]" << std::endl;
    exit(0);
}

int main(int argc, char *argv[])
{
	Face::Helpers::CmdLineArgsParser cmdLineParser(argc, argv);

    bool ok;
    std::string extractorPath = cmdLineParser.getParamValue("--extractor", ok);
    if (!ok) printHelpAndExit(argv[0]);
    std::string landmarksPath = cmdLineParser.getParamValue("--landmarks", ok);
	if (!ok) printHelpAndExit(argv[0]);
    std::string landmarksModelPath = cmdLineParser.getParamValue("--landmarksModel", ok);
	if (!ok) printHelpAndExit(argv[0]);
    std::string inputMeshPath = cmdLineParser.getParamValue("--inputMesh", ok);
	if (!ok) printHelpAndExit(argv[0]);

    bool threadPool = cmdLineParser.hasParam("--threadPool");
	if (threadPool) std::cout << "Using thread pool" << std::endl;

    float smoothCoef = 0.01;
    int smoothIterations = 10;

    Poco::Timestamp start;
    Face::Biometrics::MultiExtractor extractor(extractorPath);
    extractor.setEnableThreadPool(threadPool);
    int64 timeExtractorLoading = start.elapsed()/1000;

    start.update();
    Face::FaceData::FaceAlignerLandmark aligner(landmarksModelPath);
    int64 timeAlignerLoading = start.elapsed()/1000;

    start.update();
    Face::FaceData::Mesh inputMesh = Face::FaceData::Mesh::fromFile(inputMeshPath);
    Face::FaceData::Landmarks landmarks(landmarksPath);
    int64 timeMeshLoading = start.elapsed()/1000;

    start.update();
    Face::FaceData::SurfaceProcessor::mdenoising(inputMesh, smoothCoef, smoothIterations, smoothIterations);
    int64 timeMeshSmoothing = start.elapsed()/1000;

    start.update();
    aligner.align(inputMesh, landmarks);
    int64 timeMeshAligning = start.elapsed()/1000;

    start.update();
    Face::Biometrics::MultiTemplate featureVector = extractor.extract(inputMesh, 1, 1);
    int64 timeFeatureExtraction = start.elapsed()/1000;

    start.update();
    extractor.compare(featureVector, featureVector);
    int64 timeComparison = start.elapsed()/1000;

    std::cout << "timeExtractorLoading: " << timeExtractorLoading << std::endl;
    std::cout << "timeAlignerLoading: " << timeAlignerLoading << std::endl;
    std::cout << "timeMeshLoading: " << timeMeshLoading << std::endl;
    std::cout << "timeMeshSmoothing: " << timeMeshSmoothing << std::endl;
    std::cout << "timeMeshAligning: " << timeMeshAligning << std::endl;
    std::cout << "timeFeatureExtraction: " << timeFeatureExtraction << std::endl;
    std::cout << "timeComparison: " << timeComparison << std::endl;
}
