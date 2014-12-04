#include "faceCommon/facedata/templatematchingtrainer.h"

#include <Poco/Path.h>
#include <Poco/Timestamp.h>
#include <numeric>

#include "faceCommon/facedata/facealigner.h"
#include "faceCommon/linalg/loader.h"
#include "faceCommon/biometrics/biodataprocessing.h"
#include "faceCommon/biometrics/multiextractor.h"

using namespace Face::FaceData;

void TemplateMatchingTrainer::evaluate(const std::vector<Mesh> &alignedMeshes, const std::vector<int> &ids, const std::string &pcaDepthmapPath)
{
    cv::FileStorage storage(pcaDepthmapPath, cv::FileStorage::READ);
    auto unit = Face::Biometrics::MultiExtractor::Unit::parse("image depth scale-0.5 zpca correlation");
    unit->featureExtractor->deserialize(storage);

    int n = alignedMeshes.size();
    std::vector<Face::Biometrics::Template> templates(n);
    std::vector<Matrix> depthMaps(n);
    std::map<int, std::vector<Matrix>> depthDict;

    #pragma omp parallel for
    for (int i = 0; i < n; i++)
    {
        Face::Biometrics::MultiExtractor::ImageData d(alignedMeshes[i]);
        templates[i] = Face::Biometrics::Template(ids[i], unit->extract(d));
        depthMaps[i] = d.typeDict["depth"].clone();
    }

    for (int i = 0; i < n; i++)
    {
        depthDict[ids[i]].push_back(depthMaps[i]);
    }

    const Matrix &firstMat = (*depthDict.cbegin()).second.at(0);
    Matrix totalDiff = Matrix::zeros(firstMat.rows, firstMat.cols);
    for (auto pair : depthDict)
    {
        const auto &values = pair.second;
        Matrix diff = Matrix::zeros(values[0].rows, values[0].cols);
        int n = values.size();
        for (int i = 0; i < n - 1; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                diff += abs(values[i] - values[j]);
            }
        }
        diff /= (((n*n) - n) / 2.0);

        //double min, max; cv::minMaxIdx(diff, &min, &max);
        //cv::imshow("diff", (diff - min)/(max - min));
        //cv::waitKey();
        totalDiff += diff;
    }

    Face::Biometrics::Evaluation eval(templates, *unit->metrics);
    std::cout << "EER: " << eval.eer << std::endl;
    std::cout << "diff: " << cv::sum(totalDiff)[0] / (totalDiff.rows * totalDiff.cols) << std::endl;
}

void TemplateMatchingTrainer::evaluateBaseline(const std::string &meanForAlign, const std::string &dataPath, const std::string &pcaDepthmapPath)
{
    FaceAligner aligner(Mesh::fromFile(meanForAlign), std::string());
    std::vector<std::string> fileNames = Face::LinAlg::Loader::listFiles(dataPath, "*.binz", Face::LinAlg::Loader::Filename);
    int n = fileNames.size();
    std::vector<int> ids = Face::Biometrics::BioDataProcessing::getIds(fileNames, "-");
    std::vector<Mesh> meshes(n);
    std::vector<double> times(n);

    #pragma omp parallel for
    for (int i = 0; i < n; i++)
    {
        //if (ids[i] >= 1002) break;
        //if (ids[i] < 5000) continue;

        Mesh m = Mesh::fromFile(dataPath + Poco::Path::separator() + fileNames[i]);
        SurfaceProcessor::mdenoising(m, 0.01, 10, 0.01);

        Poco::Timestamp before;
        aligner.icpAlign(m, 50, FaceAligner::TemplateMatching);
        times[i] = before.elapsed()/1000;
        meshes[i] = m;
    }

    std::cout << "mean align time (ms) :" << (std::accumulate(times.begin(), times.end(), 0.0)/n) << std::endl;
    evaluate(meshes, ids, pcaDepthmapPath);
}

void TemplateMatchingTrainer::train(const std::string &meanForAlign, const std::string &dataPath, const std::string &pcaDepthmapPath)
{
    FaceAligner aligner(Mesh::fromFile(meanForAlign), std::string());
    std::vector<std::string> fileNames = Face::LinAlg::Loader::listFiles(dataPath, "*.binz", Face::LinAlg::Loader::BaseFilename);
    std::vector<int> ids = Face::Biometrics::BioDataProcessing::getIds(fileNames, "-");
    int n = fileNames.size();

    cv::FileStorage storage(pcaDepthmapPath, cv::FileStorage::READ);
    auto unit = Face::Biometrics::MultiExtractor::Unit::parse("image depth scale-0.5 zpca correlation");
    unit->featureExtractor->deserialize(storage);

    std::vector<cv::Size> templateSizes;
    templateSizes   /*<< cv::Size(40, 70)  << cv::Size(60, 70)*/  .push_back(cv::Size(60, 90))  /*<< cv::Size(90, 90)*/;
    std::vector<cv::Point> templateCenters;
    templateCenters /*<< cv::Point(20, 50) << cv::Point(30, 50)*/  .push_back(cv::Point(30, 60)) /*<< cv::Point(45, 60)*/;
    for (int method = cv::TM_CCOEFF_NORMED; method <= cv::TM_CCOEFF_NORMED; method++)
    {
        std::cout << "method" << method << std::endl;
        for (int type = FaceAligner::CVTemplateMatchingSettings::Mean; type <= FaceAligner::CVTemplateMatchingSettings::Mean; type++)
        {
            switch (type)
            {
            case FaceAligner::CVTemplateMatchingSettings::Texture:
                std::cout << "Texture" << std::endl;
                break;
            case FaceAligner::CVTemplateMatchingSettings::Depth:
                std::cout << "Depth" << std::endl;
                break;
            case FaceAligner::CVTemplateMatchingSettings::Mean:
                std::cout << "Mean" << std::endl;
                break;
            case FaceAligner::CVTemplateMatchingSettings::Gauss:
                std::cout << "Gauss" << std::endl;
                break;
            case FaceAligner::CVTemplateMatchingSettings::Index:
                std::cout << "Index" << std::endl;
                break;
            case FaceAligner::CVTemplateMatchingSettings::Eigen:
                std::cout << "Eigen" << std::endl;
                break;
            }

            for (unsigned int templateIndex = 0; templateIndex < templateSizes.size(); templateIndex++)
            {
                std::cout << "template " << templateSizes[templateIndex].width << " " << templateSizes[templateIndex].height << std::endl;

                std::vector<Mesh> meshes(n);
                std::vector<Landmarks> landmarks(n);
                #pragma omp parallel for
                for (int i = 0; i < n; i++)
                {
                    Mesh m = Mesh::fromFile(dataPath + Poco::Path::separator() + fileNames[i] + ".binz");
                    landmarks[i] = Landmarks(dataPath + Poco::Path::separator() + fileNames[i] + ".yml");
                    m.translate(-landmarks[i].get(Landmarks::Nosetip));
                    SurfaceProcessor::mdenoising(m, 0.01, 10, 0.01);
                    meshes[i] = m;
                }

                //qDebug() << "mean template";
                int meanTemplateCount = 50;
                cv::Mat_<float> templateImg = cv::Mat_<float>::zeros(templateSizes[templateIndex]);
                for (int i = 0; i < meanTemplateCount; i++)
                {
                    MapConverter mc;
                    Map depth = SurfaceProcessor::depthmap(meshes[i], mc, 1.0, SurfaceProcessor::ZCoord);
                    Map blurredDepth = depth;
                    blurredDepth.applyCvGaussBlur(7, 3);

                    cv::Point2d noseTip = mc.MeshToMapCoords(depth, cv::Point2d(0.0, 0.0));
                    cv::Rect roi(noseTip.x - templateCenters[templateIndex].x,
                                 noseTip.y - templateCenters[templateIndex].y,
                                 templateSizes[templateIndex].width, templateSizes[templateIndex].height);

                    cv::Mat_<float> img;

                    CurvatureStruct cs = SurfaceProcessor::calculateCurvatures(blurredDepth);
                    switch (type)
                    {
                    case FaceAligner::CVTemplateMatchingSettings::Texture:
                        SurfaceProcessor::depthmap(meshes[i], mc, 1.0, SurfaceProcessor::Texture_I).toMatrix(0, 0, 255)(roi).convertTo(img, CV_32F);
                        break;
                    case FaceAligner::CVTemplateMatchingSettings::Depth:
                        depth.values(roi).convertTo(img, CV_32F);
                        break;
                    case FaceAligner::CVTemplateMatchingSettings::Mean:
                        img = cs.meanMatrix()(roi);
                        break;
                    case FaceAligner::CVTemplateMatchingSettings::Gauss:
                        img = cs.gaussMatrix()(roi);
                        break;
                    case FaceAligner::CVTemplateMatchingSettings::Index:
                        img = cs.indexMatrix()(roi);
                        break;
                    case FaceAligner::CVTemplateMatchingSettings::Eigen:
                        img = cs.pclMatrix()(roi);
                        break;
                    }

                    templateImg += img;
                }
                templateImg /= meanTemplateCount;
                //double min, max; cv::minMaxIdx(templateImg, &min, &max);
                //cv::imshow("mean", templateImg); //(meanDepth - min)/(max - min));
                //cv::waitKey();

                aligner.cvTemplateMatchingSettings.center = templateCenters[templateIndex];
                aligner.cvTemplateMatchingSettings.comparisonMethod = method;
                aligner.cvTemplateMatchingSettings.templateImage = templateImg;
                aligner.cvTemplateMatchingSettings.inputImageType = (FaceAligner::CVTemplateMatchingSettings::InputImageType)type;

                cv::FileStorage storage("../../test/preAlignTemplate.yml", cv::FileStorage::WRITE);
                aligner.cvTemplateMatchingSettings.serialize(storage);

                //qDebug() << "align";
                std::vector<double> times(n);
                #pragma omp parallel for
                for (int i = 0; i < n; i++)
                {
                    meshes[i].translate(landmarks[i].get(Landmarks::Nosetip));

                    Poco::Timestamp before;
                    aligner.icpAlign(meshes[i], 50, FaceAligner::CVTemplateMatching);
                    times[i] = before.elapsed()/1000;
                }

                std::cout << "mean align time (ms): " << (std::accumulate(times.begin(), times.end(), 0.0)/n) << std::endl;
                evaluate(meshes, ids, pcaDepthmapPath);
            }
        }
    }
}
