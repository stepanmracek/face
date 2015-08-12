#include "evaluaterealsense.h"

#include <utility>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/biometrics/multiextractor.h"
#include "faceSensors/isensor.h"
#include "faceCommon/facedata/faceprocessor.h"

std::map<int, std::string> EvaluateRealsense::getNames()
{
    std::map<int, std::string> map;
    map[7] = "Ondra";
    map[42] = "Rene";
    map[43] = "Daniel";
    map[44] = "Torsten";
    map[45] = "Cat";
    map[46] = "Alex";
    map[47] = "Philipe";
    map[49] = "Hosteska-piha";
    map[50] = "Hosteska-rasy";
    return map;
}

double score(double distance)
{
    return -distance * 1000 + 10000;
}

void EvaluateRealsense::identification()
{
    std::string dataPath = "/mnt/data/face/realsense/IFSEC-with-id/";
    std::string classifier = "/mnt/data/face/realsense/classifiers/avg-md1-lm-svm/";

    Face::FaceData::FaceProcessor faceProcessor(new Face::FaceData::FaceAlignerLandmark());

    auto namesDict = getNames();
    std::map<int, std::vector<Face::Biometrics::MultiTemplate>> database;
    Face::Biometrics::MultiExtractor extractor(classifier);

    std::cout << "Enrollment" << std::endl;
    std::vector<std::string> enrollmentScanNames = Face::LinAlg::Loader::listFiles(dataPath, "*-mesh_enrollment_before_align_*.binz", Face::LinAlg::Loader::Filename);
    std::vector<int> enrollmentIds = Face::LinAlg::Loader::getClasses(enrollmentScanNames, "-");
    std::set<int> uniqueEnrollmentIds(enrollmentIds.begin(), enrollmentIds.end());
    size_t n = enrollmentScanNames.size();
    for (size_t i = 0; i < n; i++)
    {
        std::cout << enrollmentIds[i] << " " << namesDict[enrollmentIds[i]] << " " << enrollmentScanNames[i] << std::endl;
        Face::Sensors::Scan scan(dataPath + enrollmentScanNames[i]);
        faceProcessor.process(scan.mesh, scan.landmarks);
        database[enrollmentIds[i]].push_back(extractor.extract(scan.mesh, 0, enrollmentIds[i]));
    }

    std::cout << "Identification" << std::endl;
    std::vector<std::string> identScanNames = Face::LinAlg::Loader::listFiles(dataPath, "*-mesh_authentication_before_align_*.binz", Face::LinAlg::Loader::Filename);
    std::vector<int> identIds = Face::LinAlg::Loader::getClasses(identScanNames, "-");
    n = identScanNames.size();
    for (size_t i = 0; i < n; i++)
    {
        int id = identIds[i];
        std::string name = (namesDict.find(id) != namesDict.end() ? namesDict[id] : std::to_string(id));

        Face::Sensors::Scan probe(dataPath + identScanNames[i]);
        faceProcessor.process(probe.mesh, probe.landmarks);
        Face::Biometrics::MultiTemplate probeTemplate = extractor.extract(probe.mesh, 0, identIds[i]);

        int bestId = -1;
        std::vector<std::pair<int, double>> falseAccepts;
        double bestDistance = DBL_MAX;
        for (int enrollID : uniqueEnrollmentIds)
        {
            double d = extractor.compare(database[enrollID], probeTemplate, 1).distance;
            if (d < bestDistance)
            {
                bestDistance = d;
                bestId = enrollID;
            }

            if (d < -0.9 && id != enrollID)
            {
                falseAccepts.push_back(std::make_pair(enrollID, d));
            }
        }

        if (bestDistance < -0.9)
        {
            std::cout << name << " recognized as " << namesDict[bestId] << " with score " << score(bestDistance) << " " << std::endl;
            if (!falseAccepts.empty())
            {
                std::cout << "  potential false accepts: " << std::endl;
                for (const std::pair<int, double> fa : falseAccepts)
                {
                    std::cout << "    " << namesDict[fa.first] << " with score " << score(fa.second) << std::endl;
                }
            }
        }
        else
        {
            std::cout << name << " not in db, best score: " << bestDistance << std::endl;
        }
    }
}

void EvaluateRealsense::verification()
{
    std::string dataPath = "/mnt/data/face/realsense/IFSEC-with-id/";
    std::string classifier = "/mnt/data/face/realsense/classifiers/avg-md1-lm-svm/";

    Face::FaceData::FaceProcessor faceProcessor(new Face::FaceData::FaceAlignerLandmark());
    Face::Biometrics::MultiExtractor extractor(classifier);
    std::vector<std::string> scanNames = Face::LinAlg::Loader::listFiles(dataPath, "*.binz", Face::LinAlg::Loader::Filename);
    std::vector<int> ids = Face::LinAlg::Loader::getClasses(scanNames, "-");

    size_t n = scanNames.size();
    std::vector<Face::Biometrics::MultiTemplate> templates(n);
    #pragma omp parallel for
    for (size_t i = 0; i < n; i++)
    {
        Face::Sensors::Scan scan(dataPath + scanNames[i]);
        faceProcessor.process(scan.mesh, scan.landmarks);
        templates[i] = extractor.extract(scan.mesh, 0, ids[i]);
    }

    Face::Biometrics::Evaluation evaluation = extractor.evaluate(templates);
    evaluation.outputResults("/mnt/data/face/realsense/IFSEC-with-id/eval/eval", 30);
}
