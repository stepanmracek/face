#ifndef MULTIEXTRACTOR_H
#define MULTIEXTRACTOR_H

#include "faceCommon/linalg/common.h"
#include "faceCommon/linalg/vector.h"
#include "faceCommon/linalg/imagefilter.h"
#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceCommon/biometrics/filterfeatureextractor.h"
#include "faceCommon/biometrics/featureextractor.h"
#include "faceCommon/biometrics/scorelevefusion.h"
#include "faceCommon/biometrics/multitemplate.h"

namespace Face {
namespace Biometrics {

class ExtractorThreadPool;
class ImageDataThreadPool;

/**
 * MultiExtractor
 */
class FACECOMMON_EXPORTS MultiExtractor
{
private:
    ExtractorThreadPool *extractorThreadPool;
    ImageDataThreadPool *imageDataThreadPool;

public:
    typedef cv::Ptr<MultiExtractor> Ptr;

	class FACECOMMON_EXPORTS ImageData
    {
    public:
        std::map<std::string, Matrix> typeDict;
        Face::FaceData::Map largeDepth;
        Face::FaceData::MapConverter depthConverter;

        ImageData() { }
        ImageData(const FaceData::Mesh &mesh);
        ImageData(const ImageGrayscale &textureImage);

        static std::vector<std::string> getTypes();
    };

	class FACECOMMON_EXPORTS Unit
    {
    public:
        typedef cv::Ptr<Unit> Ptr;
        virtual ~Unit() {}

        Face::LinAlg::Metrics::Ptr metrics;
        FeatureExtractor::Ptr featureExtractor;

        static Unit::Ptr parse(const std::string &params);
        virtual std::string writeParams() const = 0;
        virtual void train(const std::vector<int> &ids, const std::vector<ImageData> &imageData) = 0;
        virtual Face::LinAlg::Vector extract(const ImageData &data) const = 0;
        double compare(const Face::LinAlg::Vector &first, const Face::LinAlg::Vector &second);
    };

    class ImageUnit : public Unit
    {
    public:
        std::string type;
        std::vector<Face::LinAlg::ImageFilter::Ptr> imageFilters;

        static Unit::Ptr parse(const std::string &params);
        std::string writeParams() const;
        void train(const std::vector<int> &ids, const std::vector<ImageData> &imageData);
        Face::LinAlg::Vector extract(const ImageData &data) const;
    };

    class CurveUnit : public Unit
    {
    public:
        std::vector<cv::Point3d> curveCenters;
        std::vector<int> radiuses;
        std::vector<int> pointCounts;

        static Unit::Ptr parse(const std::string &params);
        std::string writeParams() const;
        void train(const std::vector<int> &ids, const std::vector<ImageData> &imageData);
        Face::LinAlg::Vector extract(const ImageData &data) const;
    };

    struct ComparisonResult
    {
        ComparisonResult() : distance(0) {}
        std::vector<ScoreLevelFusionBase::Result> perReferenceResults;
        double distance;
    };

    std::vector<Unit::Ptr> units;

    cv::Ptr<ScoreLevelFusionBase> fusion;

    MultiExtractor();
    MultiExtractor(std::string directoryPath);
    virtual ~MultiExtractor();

    void setEnableThreadPool(bool enable);

    void serialize(std::string directoryPath) const;

    MultiTemplate extract(const ImageData &data, int version, int id) const;
    MultiTemplate extract(const FaceData::Mesh &mesh, int version, int id) const;
    std::vector<MultiTemplate> extract(const std::vector<FaceData::Mesh> &meshes, const std::vector<int> &ids, int version) const;

    bool checkTemplate(const MultiTemplate &t);
    ScoreLevelFusionBase::Result compare(const MultiTemplate &first, const MultiTemplate &second) const;
    ComparisonResult compare(const std::vector<MultiTemplate> &reference, const MultiTemplate &probe, int count = -1) const;

    Evaluation evaluate(const std::vector<MultiTemplate> &templates) const;
    double rankOneIdentification(const std::vector<MultiTemplate> &templates) const;
    void createPerSubjectScoreCharts(const std::vector<MultiTemplate> &templates, const std::string &pathPrefix) const;
};

}
}


#endif // MULTIEXTRACTOR_H
