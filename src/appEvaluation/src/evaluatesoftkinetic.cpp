#include "evaluatesoftkinetic.h"

#include <Poco/Path.h>
#include <Poco/NumberParser.h>
#include <Poco/StringTokenizer.h>
#include <Poco/Timestamp.h>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/facedata/templatematchingtrainer.h"
#include "faceCommon/biometrics/discriminativepotential.h"
#include "faceCommon/biometrics/eerpotential.h"
#include "faceCommon/biometrics/template.h"
#include "faceCommon/facedata/faceprocessor.h"
#include "faceCommon/linalg/loader.h"
#include "faceCommon/facedata/faceprocessor.h"

EvaluateSoftKinetic::EvaluateSoftKinetic()
{

}

void EvaluateSoftKinetic::loadDirectory()
{
	int galleryCount = 3;
	std::string dir = "C:\\data\\face\\realsense\\avg";
	std::vector<int> ids;
	std::vector<Face::FaceData::Mesh> meshes;
	Face::FaceData::FaceProcessor processor(new Face::FaceData::FaceAlignerLandmark(), 0.01, 10);

	auto baseFileNames = Face::LinAlg::Loader::listFiles(dir, "*.binz", Face::LinAlg::Loader::BaseFilename);

	std::cout << "loading meshes" << std::endl;
	for (const auto &baseFileName : baseFileNames)
	{
		auto mesh = Face::FaceData::Mesh::fromFile(dir + Poco::Path::separator() + baseFileName + ".binz");
		auto lm = Face::FaceData::Landmarks(dir + Poco::Path::separator() + baseFileName + "-landmarks.yml");
		int id = Poco::NumberParser::parse(Poco::StringTokenizer(baseFileName, "-")[0]);

		processor.process(mesh, lm);
		auto bioTemplate = extractor.extract(mesh, 0, id);

		std::cout << "references for id " << id << ": " << gallery[id].size() << std::endl;
        if (gallery[id].size() < galleryCount)
        {
			std::cout << "  -> Adding to gallery" << std::endl;
            gallery[id].push_back(bioTemplate);
        }
        else
        {
			std::cout << "  -> Adding to probes" << std::endl;
			probes.push_back(bioTemplate);
			break;
        }

		std::cout << id << " " << baseFileName << std::endl;
    }
	std::cout << gallery.size() << " " << probes.size() << std::endl;
}

void permute(std::vector<unsigned int> &values)
{
    unsigned int n = values.size();
    for (unsigned int i = 0; i < n - 1; i++)
    {
        unsigned int j = i + (rand() % (n-i));
        std::swap(values[i], values[j]);
    }
}

void EvaluateSoftKinetic::loadDirectory(const std::string &dirPath, unsigned int galleryCount, bool randomize)
{
    std::map<int, std::vector<Face::Biometrics::MultiTemplate>> allTemplates;

    std::vector<std::string> files = Face::LinAlg::Loader::listFiles(dirPath, "*.gz", Face::LinAlg::Loader::Filename);
    for (const std::string &file : files)
    {
        int id = Poco::NumberParser::parse(Poco::StringTokenizer(file, "-")[0]);
        Face::Biometrics::MultiTemplate t;
        cv::FileStorage storage(dirPath + Poco::Path::separator() + file, cv::FileStorage::READ);
        t.deserialize(storage);
        allTemplates[id].push_back(t);
    }

    for (const auto &allTemplatesPairItem : allTemplates)
    {
        const std::vector<Face::Biometrics::MultiTemplate> &templates = allTemplatesPairItem.second;
        std::vector<unsigned int> indicies;
        for (unsigned int i = 0; i < templates.size(); i++)
            indicies.push_back(i);

        if (randomize)
            permute(indicies);

        for (unsigned int i : indicies)
        {
            const auto &t = templates[i];
            if (gallery.count(t.id) < galleryCount)
            {
                gallery[t.id].push_back(t);
            }
            else
            {
                probes.push_back(t);
            }
        }
    }
}

void EvaluateSoftKinetic::evaluatePerformance()
{
	std::cout << "EvaluateSoftKinetic::evaluatePerformance()" << std::endl;

	for (const Face::Biometrics::MultiTemplate &probe : probes)
	{
		int id = probe.id;
		std::cout << id << std::endl;

		// behave as an impostor
		int bestId;
		double bestDistance = DBL_MAX;
		for (const auto &galleryPair : gallery)
		{
			int galleryId = galleryPair.first;
			if (galleryId == id) continue;

			auto compResult = extractor.compare(gallery[galleryId], probe, 1);
			double d = compResult.distance;
			if (d < bestDistance)
			{
				bestDistance = d;
				bestId = galleryId;
			}
		}

		std::cout << "  impostor:" << std::endl;
		std::cout << "    recognized as " << bestId << std::endl;
		std::cout << "    with score: " << (-1000 * bestDistance + 10000) << std::endl;

		// now behave as the genuine user
		auto compResult = extractor.compare(gallery[id], probe, 1);
		double d = compResult.distance;

		std::cout << "  genuine:" << std::endl;
		std::cout << "    scores: " << (-1000 * d + 10000) << std::endl;

		for (const auto &r : compResult.perReferenceResults)
		{
			std::cout << outScores(r.preNormalized) << std::endl;
			std::cout << outScores(r.normalized) << std::endl;
			std::cout << r.score << " " << (-1000 * r.score + 10000) << std::endl;
		}
	}
}

Face::Biometrics::ZPCACorrW EvaluateSoftKinetic::getFRGCExtractor()
{
    std::string path = "/media/data/frgc/spring2004/zbin-aligned2/";
    std::string filter = "*.binz";
    const std::vector<std::string> fileNames = Face::LinAlg::Loader::listFiles(path, filter, Face::LinAlg::Loader::Filename);

    int n = 300; //fileNames.count();
    std::vector<Face::LinAlg::Vector> trainVecs(n);

    #pragma omp parallel for
    for (int i = 0; i < n; i++)
    {
        Face::FaceData::Mesh m = Face::FaceData::Mesh::fromFile(path + Poco::Path::separator() + fileNames[i]);
        Face::Biometrics::MultiExtractor::ImageData d(m);
        trainVecs[i] = Face::LinAlg::MatrixConverter::matrixToColumnVector(Face::LinAlg::MatrixConverter::scale(d.typeDict["index"], 0.5));
    }

    Face::Biometrics::ZPCACorrW zpca(trainVecs, 0.995, trainVecs);
    cv::FileStorage storage("frgc-index-zpca", cv::FileStorage::WRITE);
    zpca.extractor.serialize(storage);
    return zpca;
}

std::vector<Face::LinAlg::Vector> EvaluateSoftKinetic::smoothMeshes(const std::vector<Face::FaceData::Mesh> &in, float smoothCoef,
	int smoothIters, const Face::FaceData::FaceAlignerIcp &aligner, int icpIters, bool useMdenoise)
{
    int n = in.size();
    std::vector<Face::LinAlg::Vector> out(n);
    std::vector<double> sumTime(n);

    #pragma omp parallel for
    for (int i = 0; i < n; i++)
    {
        Face::FaceData::Mesh m = in[i];
        Poco::Timestamp stamp;        if (smoothIters > 0)
        {
            if (useMdenoise)
                Face::FaceData::SurfaceProcessor::mdenoising(m, smoothCoef, smoothIters, smoothIters);
            else
                Face::FaceData::SurfaceProcessor::zsmooth(m, smoothCoef, smoothIters);
        }
        sumTime[i] = stamp.elapsed()/1000;
		aligner.align(m, icpIters, Face::FaceData::FaceAlignerIcp::TemplateMatching);

        Face::Biometrics::MultiExtractor::ImageData d(m);
        Matrix scaled = Face::LinAlg::MatrixConverter::scale(d.typeDict["index"], 0.5);
        out[i] = Face::LinAlg::MatrixConverter::matrixToColumnVector(scaled);
    }

    double sum = 0;
    for (double t : sumTime)
    {
        sum += t;
    }
    std::cout << (sum/n) << std::endl;

    return out;
}

void EvaluateSoftKinetic::evaluateOptimalSmoothing()
{
	Face::FaceData::FaceAlignerIcp aligner(Face::FaceData::Mesh::fromFile("../../test/meanForAlign.obj"), std::string());
    Face::Biometrics::ZScorePCAExtractor extractor;
    cv::FileStorage storage("frgc-index-zpca", cv::FileStorage::READ);
    extractor.deserialize(storage);
    int icpIters = 50;

    std::string path = "/mnt/data/face/softkinetic/epar-train";
    std::string filter = "*.binz";
    std::vector<int> ids;
    std::vector<Face::FaceData::Mesh> meshes;
    const std::vector<std::string> fileNames = Face::LinAlg::Loader::listFiles(path, filter, Face::LinAlg::Loader::Filename);
    int n = fileNames.size();
    ids.resize(n);
    meshes.resize(n);

    #pragma omp parallel for
    for (int i = 0; i < n; i++)
    {
        Face::FaceData::Mesh m = Face::FaceData::Mesh::fromFile(path + Poco::Path::separator() + fileNames[i]);
        ids[i] = Poco::NumberParser::parse(Poco::StringTokenizer(fileNames[i], "-")[0]);
        meshes[i] = m;
    }

    std::vector<Face::LinAlg::Vector> vecs = smoothMeshes(meshes, 0, 0, aligner, icpIters, true);
    std::cout << "no smoothing: " << Face::Biometrics::Evaluation(vecs, ids, extractor, Face::LinAlg::CorrelationMetric()).eer << std::endl;

    std::vector<int> smoothItersVec; smoothItersVec.push_back(5); smoothItersVec.push_back(10); smoothItersVec.push_back(20);
    std::vector<float> smoothCoefsVec; smoothCoefsVec.push_back(0.01f); smoothCoefsVec.push_back(0.02f); smoothCoefsVec.push_back(0.04f);
    for(int smoothIters : smoothItersVec)
    {
        for(float smoothCoef : smoothCoefsVec)
        {
            std::vector<Face::LinAlg::Vector> vecs = smoothMeshes(meshes, smoothCoef, smoothIters, aligner, icpIters, true);
            std::cout << "mdenoise " << smoothIters << " " << smoothCoef << " "
                      << Face::Biometrics::Evaluation(vecs, ids, extractor, Face::LinAlg::CorrelationMetric()).eer << std::endl;
        }
    }

    smoothItersVec.clear(); smoothItersVec.push_back(1); smoothItersVec.push_back(2); smoothItersVec.push_back(5);
    smoothCoefsVec.clear(); smoothCoefsVec.push_back(0.2f); smoothCoefsVec.push_back(0.5f); smoothCoefsVec.push_back(1.0f);
    for(int smoothIters : smoothItersVec)
    {
        for (float smoothCoef : smoothCoefsVec)
        {
            std::vector<Face::LinAlg::Vector> vecs = smoothMeshes(meshes, smoothCoef, smoothIters, aligner, icpIters, false);
            std::cout << "zsmooth " << smoothIters << " " << smoothCoef << " "
                      << Face::Biometrics::Evaluation(vecs, ids, extractor, Face::LinAlg::CorrelationMetric()).eer << std::endl;
        }
    }
}

void EvaluateSoftKinetic::stats()
{
    std::string path = "/home/stepo/data/face/softkinetic/epar";
    std::vector<std::string> fileNames = Face::LinAlg::Loader::listFiles(path, "*.binz", Face::LinAlg::Loader::Filename);

    auto ids = Face::LinAlg::Loader::getClasses(fileNames, "-");
    std::set<int> uniqueIds(ids.begin(), ids.end());
    std::cout << uniqueIds.size() << std::endl;
}

void EvaluateSoftKinetic::evaluateAging()
{
    std::string extractorPath = "/home/stepo/build/face/release-qt5/appAutoTrainer/epar/extractor/04/";
    std::string dataPath = "/mnt/data/face/softkinetic/epar/";
    unsigned int minScans = 15;
    std::map<int, std::vector<std::string>> dict;
    Face::Biometrics::MultiExtractor extractor(extractorPath);
	Face::FaceData::FaceAlignerIcp aligner(Face::FaceData::Mesh::fromFile("../../test/meanForAlign.obj"), std::string());

    std::vector<std::string> allFiles = Face::LinAlg::Loader::listFiles(dataPath, "*.binz", Face::LinAlg::Loader::Filename);
    for (const std::string &fn : allFiles)
    {
        int id = Poco::NumberParser::parse(Poco::StringTokenizer(fn, "*")[0]);
        dict[id].push_back(fn);
    }

    std::vector<int> toRemove;
    for (const auto &pair : dict)
    {
        if (dict[pair.first].size() < minScans)
            toRemove.push_back(pair.first);
    }
    for (int id : toRemove)
        dict.erase(id);

    for (const auto &pair : dict)
    {
        const std::vector<std::string> &fileNames = pair.second;
        std::vector<Face::Biometrics::MultiTemplate> reference;
        for (unsigned int i = 0; i < fileNames.size(); i++)
        {
            auto mesh = Face::FaceData::Mesh::fromFile(dataPath + fileNames[i]);
            Face::FaceData::SurfaceProcessor::mdenoising(mesh, 0.01, 10, 10);
			aligner.align(mesh, 100, Face::FaceData::FaceAlignerIcp::TemplateMatching);
            auto t = extractor.extract(mesh, 0, pair.first);

            if (i < 4)
            {
                reference.push_back(t);
                continue;
            }

            double score = extractor.compare(reference, t).distance;
            std::cout << pair.first << " " << score << std::endl;
        }
    }
}

void EvaluateSoftKinetic::problematicScans()
{
    std::string galleryPath = "/home/stepo/storage/gallery/";
    std::string probesPath = "/home/stepo/storage/probes/";

    Face::Biometrics::MultiExtractor extractor("/mnt/data/face/realsense/classifiers/avg-md1-lm-svm/");
    Face::FaceData::FaceProcessor processor(new Face::FaceData::FaceAlignerLandmark());

    std::cout << "Loading probe" << std::endl;
    Face::FaceData::Mesh probeMesh = Face::FaceData::Mesh::fromFile(probesPath + "mesh_authentication_before_align_2015-06-10T09-06-03_02-00.binz");
    Face::FaceData::Landmarks probeLm(probesPath + "mesh_authentication_before_align_2015-06-10T09-06-03_02-00-landmarks.yml");
    processor.process(probeMesh, probeLm);
    Face::Biometrics::MultiTemplate probeTemplate = extractor.extract(probeMesh, 0, 0);

    std::cout << "Loading gallery" << std::endl;
    std::vector<std::string> galleryBaseNames = Face::LinAlg::Loader::listFiles(galleryPath, "*.binz", Face::LinAlg::Loader::BaseFilename);
    for (const std::string & f: galleryBaseNames)
    {
        std::cout << f << ": ";

        Face::FaceData::Mesh galleryMesh = Face::FaceData::Mesh::fromFile(galleryPath + f + ".binz");
        Face::FaceData::Landmarks galleryLm(galleryPath + f + "-landmarks.yml");
        processor.process(galleryMesh, galleryLm);
        Face::Biometrics::MultiTemplate galleryTemplate = extractor.extract(galleryMesh, 0, 0);

        std::cout << extractor.compare(probeTemplate, galleryTemplate).score << std::endl;
        std::cout << "  coverage: " << galleryTemplate.depthCoverage << std::endl;
    }
}

void EvaluateSoftKinetic::evaluateMetrics()
{
    Face::LinAlg::CosineMetric m;
    Poco::Timestamp stamp;
    for (const auto &galleryPair : gallery)
    {
        unsigned int id = galleryPair.first;
        const Face::Biometrics::MultiTemplate &reference = galleryPair.second.at(0);
        for (const Face::Biometrics::MultiTemplate &probe : probes)
        {
            double d;
            for (unsigned int i = 0; i < reference.featureVectors.size(); i++)
            {
                d += m.distance(reference.featureVectors[i], probe.featureVectors[i]);
            }
        }
    }
    std::cout << (stamp.elapsed()/1000) << std::endl;
}

void EvaluateSoftKinetic::evaluateAlignBaseline()
{
    Face::FaceData::TemplateMatchingTrainer::evaluateBaseline("../../test/meanForAlign.obj",
                                                              "/home/stepo/data/face/softkinetic/epar",
                                                              "../../test/frgc-depth-zpca");
}

void EvaluateSoftKinetic::evaluateCVAlign()
{
    Face::FaceData::TemplateMatchingTrainer::train("../../test/meanForAlign.obj",
                                                   "/home/stepo/data/face/softkinetic/epar",
                                                   "../../test/frgc-depth-zpca");
}

void EvaluateSoftKinetic::deviceOpenCVtest()
{
    Face::Biometrics::MultiExtractor extractor("../../test/softKineticExtractor");
    std::vector<Face::Biometrics::MultiTemplate> references;
    for (int i = 1; i <= 4; i++)
    {
        Face::FaceData::Mesh m = Face::FaceData::Mesh::fromFile("../../test/test01/ref" + std::to_string(i) + ".bin");
        references.push_back(extractor.extract(m, 0, 0));
    }

    Face::FaceData::Mesh m = Face::FaceData::Mesh::fromFile("../../test/test01/probe.bin");
    Face::Biometrics::MultiTemplate probe = extractor.extract(m, 0, 0);

    auto result = extractor.compare(references, probe, 2);
    for (const Face::Biometrics::ScoreLevelFusionBase::Result &r : result.perReferenceResults)
    {
        std::cout << r.score << std::endl;
        //qDebug() << r.preNormalized;
        //qDebug() << r.normalized;
        std::cout << "------------" << std::endl;
    }
}

void EvaluateSoftKinetic::evaluateVariability()
{
    std::string dataPath = "/mnt/data/face/softkinetic/epar/";
	Face::FaceData::FaceAlignerIcp aligner(Face::FaceData::Mesh::fromFile("../../test/meanForAlign.obj"), "../../test/preAlignTemplate.yml");

    std::vector<std::string> types = Face::Biometrics::MultiExtractor::ImageData::getTypes();
    std::map<std::string, std::vector<Face::Biometrics::Template>> templates;
    std::vector<std::string> files = Face::LinAlg::Loader::listFiles(dataPath, "*.binz", Face::LinAlg::Loader::Filename);
    //files.resize(50);
    int cols = 0;
    for (const std::string &fn : files)
    {
        int id = Poco::NumberParser::parse(Poco::StringTokenizer(fn, "-")[0]);
        std::cout << id << " " << fn << std::endl;
        auto mesh = Face::FaceData::Mesh::fromFile(dataPath + fn);
        Face::FaceData::SurfaceProcessor::mdenoising(mesh, 0.01, 10, 10);
		aligner.align(mesh, 100, Face::FaceData::FaceAlignerIcp::TemplateMatching);
        Face::Biometrics::MultiExtractor::ImageData imgData(mesh);
        if (cols == 0) cols = imgData.typeDict["depth"].cols;

        for (const std::string type : types)
        {
            templates[type].push_back(Face::Biometrics::Template(id, Face::LinAlg::MatrixConverter::matrixToColumnVector(imgData.typeDict[type])));
        }
    }

    for (std::string type : types)
    {
        std::cout << type << " dp" << std::endl;
        Face::Biometrics::DiscriminativePotential dp(templates[type]);
        Face::LinAlg::Vector weightsVector = dp.createWeights();
        Matrix weights = Face::LinAlg::MatrixConverter::columnVectorToMatrix(weightsVector, cols);
        cv::imwrite("dp-" + type + ".png", weights*255);

        std::cout << type << " ep" << std::endl;
        Face::Biometrics::DiscriminativePotential ep(templates[type]);
        weightsVector = ep.createWeights();
        weights = Face::LinAlg::MatrixConverter::columnVectorToMatrix(weightsVector, cols);
        cv::imwrite("ep-" + type + ".png", weights*255);
    }
}

int EvaluateSoftKinetic::evaluateJenda(int /*argc*/, char */*argv*/[])
{
    //QApplication app(argc, argv);
    //Face::GUI::GLWidget widget;

    std::string path = "../../test/jenda/";
    auto fileNames = Face::LinAlg::Loader::listFiles(path, "*.binz", Face::LinAlg::Loader::AbsoluteFull);

    Face::Biometrics::MultiExtractor extractor("../../test/softKineticExtractor");
    //Face::FaceData::FaceAligner aligner(Face::FaceData::Mesh::fromFile("../../test/meanForAlign.obj"),
    //                                    "../../test/preAlignTemplate.yml");

    auto reference = extractor.extract(Face::FaceData::Mesh::fromFile(fileNames[0]), 1, 1);
    for (unsigned int i = 1; i < fileNames.size(); i++)
    {
        auto probe = extractor.extract(Face::FaceData::Mesh::fromFile(fileNames[i]), 1, 1);
        auto result = extractor.compare(reference, probe);
        std::cout << result.score << std::endl; // << result.normalized;
    }

    return 0;
}
