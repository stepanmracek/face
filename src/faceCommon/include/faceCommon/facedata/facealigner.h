#pragma once

#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/landmarks.h"
#include "faceCommon/linalg/procrustes.h"
#include "faceCommon/linalg/iserializable.h"
#include "faceCommon/settings/settings.h"

namespace Face {
namespace FaceData {

class NearestPointsThreadPool;

/*class FACECOMMON_EXPORTS FaceAligner
{
public:
    typedef cv::Ptr<FaceAligner> Ptr;   

	virtual ~FaceAligner() {}
};*/

class FACECOMMON_EXPORTS FaceAlignerLandmark //: public FaceAligner
{
public:
	typedef cv::Ptr<FaceAlignerLandmark> Ptr;

	FaceAlignerLandmark(const Landmarks &referenceLandmarks = Landmarks(Face::Settings::instance().settingsMap[Face::Settings::MeanFaceModelLandmarksPathKey].convert<std::string>()));

	void align(Mesh &face, Landmarks &landmarks) const;

private:
	const Face::FaceData::Landmarks referenceLandmarks;

};

class FACECOMMON_EXPORTS FaceAlignerIcp //: public FaceAligner
{
public:
	typedef cv::Ptr<FaceAlignerIcp> Ptr;

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

	enum PreAlignTransform {
		None = 0,
		NoseTipDetection = 1,
		MaxZ = 2,
		Centralize = 3,
		TemplateMatching = 4,
		CVTemplateMatching = 5
	};

	CVTemplateMatchingSettings cvTemplateMatchingSettings;

	FaceAlignerIcp(const Mesh &referenceFace = Mesh::fromFile(Face::Settings::instance().settingsMap[Face::Settings::MeanFaceModelPathKey]),
		const std::string &templateMatchingFilePath = Face::Settings::instance().settingsMap[Face::Settings::PreAlignTemplatePathKey]);
	
	~FaceAlignerIcp();

	void setEnableThreadPool(bool enable);

	void align(Mesh &face, int maxIterations, PreAlignTransform preAlignTransform) const;

private:
	const Mesh referenceFace;
	NearestPointsThreadPool *threadPool;
	const Face::FaceData::Landmarks referenceLandmarks;

	void preAlign(Mesh &face, PreAlignTransform preAlignTransform) const;
	void alignMaxZ(Mesh &face) const;
	void alignCentralize(Mesh &face) const;
	void alignTemplateMatching(Mesh &face) const;
	void alignCVTemplateMatching(Mesh &face) const;
};

}
}
