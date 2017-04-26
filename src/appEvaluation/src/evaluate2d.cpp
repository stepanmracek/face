#include "evaluate2d.h"

#include <QApplication>
#include <QInputDialog>
#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/objectdetection/detector.h"
#include "faceCommon/facedata/facenormalization.h"
#include "faceCommon/facedata/landmarkdetector.h"
#include "faceCommon/biometrics/biodataprocessing.h"
#include "faceCommon/biometrics/multibiomertricsautotuner.h"
#include "faceCommon/biometrics/scorelevefusion.h"

using namespace Face::Biometrics;

void Evaluate2D::createNormalizedImages()
{
    static const std::string inputPath = "/mnt/data/face/frgc/spring2004/ppm/";
    static const std::string outputPath = "/mnt/data/face/frgc/spring2004/ppm/roi/";

    std::vector<std::string> fileNames = Face::LinAlg::Loader::listFiles(inputPath, "*.ppm", Face::LinAlg::Loader::Filename);
    size_t n = fileNames.size();

    Face::ObjectDetection::DlibDetector faceDetector;
    Face::FaceData::LandmarkDetector lmDetector;
    Face::FaceData::FaceNormalization normalizer;

    for (size_t i = 0; i < n; i++)
    {
        ImageGrayscale img = cv::imread(inputPath + fileNames[i], cv::IMREAD_GRAYSCALE);
        std::cout << (inputPath+fileNames[i]) << " " << img.cols << "x" << img.rows << std::endl;

        ImageGrayscale half;
        cv::resize(img, half, cv::Size(img.cols/2, img.rows/2));

        std::vector<cv::Rect> faces = faceDetector.detect(half);
        if (faces.size() > 0)
        {
            cv::Rect &roi = faces.front();
            std::vector<cv::Point2d> landmarks = lmDetector.get(half, roi);
            if (landmarks.size() > 0)
            {
                for (int j = 0; j < landmarks.size(); j++)
                    landmarks[j] = landmarks[j]*2;
                ImageGrayscale normalized = normalizer.normalize(img, landmarks);

                cv::imwrite(outputPath + fileNames[i], normalized);
            }
        }
    }
}

void Evaluate2D::trainExtractor()
{
    static const std::string inputPath = "/mnt/data/face/frgc/spring2004/ppm/roi/";

    std::cout << "Loading pca train data" << std::endl;
    auto pcaTrainData = MultiBiomertricsAutoTuner::Input::fromDirectoryWithTextureImages(inputPath, "d", 300, 0);
    std::cout << "Loading fusion train data" << std::endl;
    auto fusionTrainData = MultiBiomertricsAutoTuner::Input::fromDirectoryWithTextureImages(inputPath, "d", 300, 300);

    std::string unitsPath("/home/stepo/git/tbs-devices/3D_Face/facelib/test/units-texture");
    MultiBiomertricsAutoTuner::Settings settings(ScoreSVMFusion::name() + "-" + ScoreNormalizerMean::name(), unitsPath);
    std::cout << "Training from " << settings.params.size() << " units" << std::endl;
    auto extractor = MultiBiomertricsAutoTuner::trainWithWrapper(pcaTrainData, fusionTrainData, settings);
    extractor->serialize("/mnt/data/face/test/2d-extractor");
}

void Evaluate2D::evaluateExtractor()
{
    MultiExtractor extractor("/mnt/data/face/test/2d-extractor");

    static const std::string inputPath = "/mnt/data/face/frgc/spring2004/ppm/roi/";
    std::vector<std::string> fileNames = Face::LinAlg::Loader::listFiles(inputPath, "*.ppm", Face::LinAlg::Loader::Filename);
    int N = fileNames.size();
    std::vector<MultiTemplate> templates;

    //#pragma omp parallel for
    for (int i = 600; i < N; i++)
    {
        ImageGrayscale img = cv::imread(inputPath + fileNames[i], cv::IMREAD_GRAYSCALE);
        MultiExtractor::ImageData data(img);

        Poco::StringTokenizer items(fileNames[i], "d");
        int id = Poco::NumberParser::parse(items[0]);
        std::cout << id << std::endl;

        auto t = extractor.extract(data, 0, id);
        templates.push_back(t);
    }

    auto eval = extractor.evaluate(templates);
    eval.printStats();
    eval.outputResults("/mnt/data/face/test/eval-face2d/face2d", 50);
}

void Evaluate2D::benchmark(int argc, char *argv[])
{
    cv::VideoCapture capture(0);
    Face::ObjectDetection::DlibDetector faceDetector;
    Face::FaceData::LandmarkDetector lmDetector;
    Face::FaceData::FaceNormalization normalizer;
    MultiExtractor extractor("/mnt/data/face/test/2d-extractor");
    extractor.setEnableThreadPool(true);

    ImageBGR frameBgr;
    ImageGrayscale frame;
    ImageGrayscale half;

    Poco::Timestamp::TimeDiff detectionTime = 0;
    Poco::Timestamp::TimeDiff landmarksTime = 0;
    Poco::Timestamp::TimeDiff normalizationTime = 0;
    Poco::Timestamp::TimeDiff extractionTime = 0;
    Poco::Timestamp::TimeDiff comparisonTime = 0;
    int counter = 0;

    std::vector<MultiTemplate> templates;
    static const std::string inputPath = "/mnt/data/face/frgc/spring2004/ppm/roi/";
    std::cout << "Loading templates" << std::endl;
    auto inputData = MultiBiomertricsAutoTuner::Input::fromDirectoryWithTextureImages(inputPath, "d", 300, 600);
    for (const MultiExtractor::ImageData &d : inputData.imageData)
        templates.push_back(extractor.extract(d, 0, 0));

    while (capture.read(frameBgr))
    {
        cv::cvtColor(frameBgr, frame, cv::COLOR_BGR2GRAY);
        cv::resize(frame, half, cv::Size(frame.cols/2, frame.rows/2));

        Poco::Timestamp detectStamp;
        std::vector<cv::Rect> faceRegions = faceDetector.detect(half);
        auto detectElapsed = detectStamp.elapsed();

        if (faceRegions.size() > 0)
        {
            cv::Rect &roi = faceRegions.front();

            Poco::Timestamp lmStamp;
            std::vector<cv::Point2d> landmarks = lmDetector.get(half, roi);
            auto lmElapsed = lmStamp.elapsed();
            if (landmarks.size() > 0)
            {
                for (size_t i = 0; i < landmarks.size(); i++)
                    landmarks[i] = landmarks[i]*2;

                Poco::Timestamp normalizeStamp;
                ImageGrayscale faceImage = normalizer.normalize(frame, landmarks);
                auto normalizeElapsed = normalizeStamp.elapsed();

                Poco::Timestamp extractionStamp;
                MultiExtractor::ImageData data(faceImage);
                MultiTemplate t = extractor.extract(data, 0, 0);
                auto extractionElapsed = extractionStamp.elapsed();

                Poco::Timestamp comparisonStamp;
                for (const auto &mockTemplate : templates)
                {
                    extractor.compare(t, mockTemplate);
                }
                auto comparisonElapsed = comparisonStamp.elapsed();

                counter++;
                detectionTime += detectElapsed;
                landmarksTime += lmElapsed;
                normalizationTime += normalizeElapsed;
                extractionTime += extractionElapsed;
                comparisonTime += comparisonElapsed;

                double fps = 1000000.0 / (detectStamp.elapsed());
                cv::putText(frameBgr, std::to_string(fps), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, 0, 1, CV_AA);
                //std::cout << fps << std::endl;
                cv::imshow("frame", frameBgr);
            }
        }

        char c = cv::waitKey(1);
        if (c == 27 || counter == 500) break;
    }

    detectionTime /= counter*1000;
    landmarksTime /= counter*1000;
    normalizationTime /= counter*1000;
    extractionTime /= counter*1000;
    comparisonTime /= counter*1000;

    std::cout << "detectionTime " << detectionTime << std::endl;
    std::cout << "landmarksTime " << landmarksTime << std::endl;
    std::cout << "normalizationTime " << normalizationTime << std::endl;
    std::cout << "extractionTime " << extractionTime << std::endl;
    std::cout << "comparisonTime " << comparisonTime << std::endl;
}

void Evaluate2D::realtime(int argc, char *argv[])
{
    double threshold = -5;
    QApplication app(argc, argv);
    cv::VideoCapture capture(0);
    Face::ObjectDetection::DlibDetector faceDetector;
    Face::FaceData::LandmarkDetector lmDetector;
    Face::FaceData::FaceNormalization normalizer;
    MultiExtractor extractor("/mnt/data/face/test/2d-extractor");
    extractor.setEnableThreadPool(true);

    ImageBGR frameBgr;
    ImageGrayscale frame;
    ImageGrayscale half;

    std::vector<MultiTemplate> templates;

    while (capture.read(frameBgr))
    {
        cv::cvtColor(frameBgr, frame, cv::COLOR_BGR2GRAY);
        cv::resize(frame, half, cv::Size(frame.cols/2, frame.rows/2));

        std::vector<cv::Rect> faceRegions = faceDetector.detect(half);
        MultiTemplate t;
        if (faceRegions.size() > 0)
        {
            cv::Rect &roi = faceRegions.front();

            std::vector<cv::Point2d> landmarks = lmDetector.get(half, roi);
            if (landmarks.size() > 0)
            {
                for (size_t i = 0; i < landmarks.size(); i++)
                    landmarks[i] = landmarks[i]*2;

                ImageGrayscale faceImage = normalizer.normalize(frame, landmarks);
                MultiExtractor::ImageData data(faceImage);
                t = extractor.extract(data, 1, 0);

                double minDistance = DBL_MAX;
                int minID = -1;
                for (const MultiTemplate &reference : templates)
                {
                    double d = extractor.compare(t, reference).score;
                    if (d < minDistance && d < threshold)
                    {
                        minDistance = d;
                        minID = reference.id;
                    }
                }

                auto doubleRect = [] (const cv::Rect &in) {
                    return cv::Rect(in.x*2, in.y*2, in.width*2, in.height*2);
                };

                if (minID == -1)
                {
                    cv::rectangle(frameBgr, doubleRect(roi), cv::Scalar(0, 0, 255));
                }
                else
                {
                    auto r = doubleRect(roi);
                    cv::rectangle(frameBgr, r, cv::Scalar(0, 255, 0));
                    cv::putText(frameBgr, std::to_string(minID) + ": " + std::to_string(minDistance), r.tl(), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA);
                }
            }
        }

        cv::imshow("frame", frameBgr);
        char c = cv::waitKey(1);

        if (c == ' ' && t.version > 0)
        {
            bool ok;
            int id = QInputDialog::getInt(0, "ID", "ID", 1, 1, INT_MAX, 1, &ok);
            if (ok) {
                t.id = id;
                templates.push_back(t);
            }
        }
        if (c == 27) break;
    }
}
