#ifndef FACEALIGNER_H
#define FACEALIGNER_H

#include "mesh.h"
#include "landmarks.h"
#include "nearestpointsthreadpool.h"
#include "faceCommon/linalg/procrustes.h"
#include "faceCommon/linalg/iserializable.h"

namespace Face {
namespace FaceData {

class FACECOMMON_EXPORTS FaceAligner
{
public:
    class CVTemplateMatchingSettings : public Face::LinAlg::ISerializable
    {
    public:
        enum InputImageType { Texture, Depth, Mean, Gauss, Index, Eigen };
        int comparisonMethod;
        InputImageType inputImageType;
        cv::Mat_<float> templateImage;
        cv::Point center;

        void serialize(cv::FileStorage &storage) const;
        void deserialize(cv::FileStorage &storage);
    };

    enum PreAlignTransform { None, NoseTipDetection, MaxZ, Centralize, TemplateMatching, CVTemplateMatching };

    typedef cv::Ptr<FaceAligner> Ptr;

    CVTemplateMatchingSettings cvTemplateMatchingSettings;

private:
    NearestPointsThreadPool *threadPool;

    void preAlign(Mesh &face, PreAlignTransform preAlignTransform) const;
    void alignNoseTip(Mesh &face) const;
    void alignMaxZ(Mesh &face) const;
    void alignCentralize(Mesh &face) const;
    void alignTemplateMatching(Mesh &face) const;
    void alignCVTemplateMatching(Mesh &face) const;

public:
    Mesh referenceFace;

    FaceAligner() : threadPool(0) {}
    FaceAligner(const Mesh &referenceFace, const std::string &templateMatchingFilePath);
    virtual ~FaceAligner();

    void icpAlign(Mesh &face, int maxIterations, PreAlignTransform preAlignTransform) const;

    void setEnableThreadPool(bool enable);
};

}
}

#endif // FACEALIGNER_H
