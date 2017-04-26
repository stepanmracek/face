#include "evaluatefeaturestability.h"

#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/linalg/matrixconverter.h"
#include "faceCommon/helpers/frgcutils.h"
#include "faceCommon/facedata/mesh.h"
#include "faceCommon/biometrics/multiextractor.h"
#include "faceCommon/biometrics/discriminativepotential.h"
#include "faceCommon/biometrics/template.h"

void EvaluateFeatureStability::evaluate()
{
    std::vector<std::string> types;
    //types = Face::Biometrics::MultiExtractor::ImageData::getTypes();
    types.push_back("texture");
    for (std::string type : types)
    {
        std::string frgcDir = "/home/stepo/data/frgc/spring2004/zbin-aligned2/";
        std::vector<std::string> fileNames = Face::LinAlg::Loader::listFiles(frgcDir, "*.binz", Face::LinAlg::Loader::Filename);

        int trainSize = Face::Helpers::FRGCUtils::Spring2004::part1Size();
        std::vector<int> trainIds(trainSize);
        std::vector<Face::Biometrics::Template> rawTrainTemplates(trainSize);
        std::vector<Face::LinAlg::Vector> rawTrainVectors(trainSize);
        std::vector<Face::LinAlg::Vector> rawDiscriminativeTrainVectors(trainSize);

        Face::Biometrics::ZScorePCAExtractor plainExtractor;
        Face::Biometrics::ZScorePCAExtractor dpExtractor;

        #pragma omp parallel for
        for (int i = 0; i < trainSize; i++)
        {
            //qDebug() << i << "/" << trainSize;
            int id = Poco::NumberParser::parse(Poco::StringTokenizer(fileNames[i], "d")[0]);
            auto m = Face::FaceData::Mesh::fromFile(frgcDir + fileNames[i]);
            Face::Biometrics::MultiExtractor::ImageData imgData(m);
            auto depthVec = Face::LinAlg::MatrixConverter::matrixToColumnVector(imgData.typeDict[type]);
            rawTrainTemplates[i] = Face::Biometrics::Template(id, depthVec);
            rawTrainVectors[i] = depthVec;
            trainIds[i] = id;
        }

        Face::Biometrics::DiscriminativePotential rawDP(rawTrainTemplates);
        cv::imwrite(type + ".png",
                    Face::LinAlg::MatrixConverter::columnVectorToMatrix(rawDP.createWeights(), 100)*255);
        //cv::waitKey(0);

        #pragma omp parallel for
        for (int i = 0; i < trainSize; i++)
        {
            rawDiscriminativeTrainVectors[i] = rawTrainVectors[i].mul(rawDP.createWeights());
        }

        plainExtractor.train(trainIds, rawTrainVectors);
        dpExtractor.train(trainIds, rawDiscriminativeTrainVectors);

        auto trainProjectedTemplates = Face::Biometrics::Template::joinVectorsAndClasses(plainExtractor.batchExtract(rawTrainVectors), trainIds);
        Face::Biometrics::DiscriminativePotential projectedDP(trainProjectedTemplates);

        int testSize = Face::Helpers::FRGCUtils::Spring2004::part2Size();
        std::vector<int> testIds(testSize);
        std::vector<Face::LinAlg::Vector> rawTestVectors(testSize);
        std::vector<Face::LinAlg::Vector> rawDiscriminativeTestVectors(testSize);

        #pragma omp parallel for
        for (int i = 0; i < testSize; i++)
        {
            //qDebug() << i << "/" << testSize;
            int findex = i + Face::Helpers::FRGCUtils::Spring2004::part2Start();
            int id = Poco::NumberParser::parse(Poco::StringTokenizer(fileNames[i], "d")[0]);
            auto m = Face::FaceData::Mesh::fromFile(frgcDir + fileNames[findex]);
            Face::Biometrics::MultiExtractor::ImageData imgData(m);
            auto depthVec = Face::LinAlg::MatrixConverter::matrixToColumnVector(imgData.typeDict[type]);

            testIds[i] = id;
            rawTestVectors[i] = depthVec;
            rawDiscriminativeTestVectors[i] = depthVec.mul(rawDP.createWeights());

            if (i == 0)
            {
                auto weighted = Face::LinAlg::MatrixConverter::columnVectorToMatrix(rawDiscriminativeTestVectors[0], 100);
                cv::imwrite("weighted.png", weighted*255);
                cv::imwrite("original.png", imgData.typeDict[type]*255);
                exit(0);
            }
        }

        std::cout << "type" << std::endl;
        std::cout << "plain " << Face::Biometrics::Evaluation(rawTestVectors, testIds, plainExtractor, Face::LinAlg::CorrelationMetric()).eer << std::endl;
        std::cout << "dp " << Face::Biometrics::Evaluation(rawDiscriminativeTestVectors, testIds, dpExtractor, Face::LinAlg::CorrelationMetric()).eer << std::endl;

        Face::LinAlg::CorrelationWeightedMetric corW;
        corW.w = projectedDP.createWeights();
        std::cout << "projected " << Face::Biometrics::Evaluation(rawTestVectors, testIds, plainExtractor, corW).eer << std::endl;
    }
}
