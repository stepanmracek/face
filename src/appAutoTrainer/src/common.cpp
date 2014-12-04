#include "common.h"

#include <Poco/Path.h>
#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceCommon/facedata/mdenoise/mdenoise.h"

using namespace Face::AutoTrainer;

void Common::loadMeshes(const std::string &dir, const Face::FaceData::FaceAligner &aligner,
                        std::vector<int> &ids, std::vector<Face::FaceData::Mesh> &meshes, int icpIterations,
                        int smoothIterations, float smoothCoef, const std::string &idSeparator)
{
    std::vector<std::string> filters;
    filters.push_back("*.binz");
    filters.push_back("*.bin");
    filters.push_back("*.obj");

    const std::vector<std::string> fileNames = Face::LinAlg::Loader::listFiles(dir, filters, Face::LinAlg::Loader::Filename);
    int n = fileNames.size();
    ids.resize(n);
    meshes.resize(n);

    #pragma omp parallel for
    for (int i = 0; i < n; i++)
    {
        Face::FaceData::Mesh m = Face::FaceData::Mesh::fromFile(dir + Poco::Path::separator() + fileNames[i]);

        if (smoothIterations > 0)
        {
            Face::FaceData::SurfaceProcessor::mdenoising(m, smoothCoef, smoothIterations, smoothIterations);
        }

        if (icpIterations > 0)
        {
            aligner.icpAlign(m, icpIterations, Face::FaceData::FaceAligner::CVTemplateMatching);
        }

        ids[i] = Poco::NumberParser::parse(Poco::StringTokenizer(fileNames[i], idSeparator)[0]);
        meshes[i] = m;
    }
}
