#include "faceCommon/facedata/facealigner.h"

#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/landmarkdetector.h"
#include "faceCommon/linalg/procrustes.h"
#include "faceCommon/linalg/kernelgenerator.h"
#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceCommon/facedata/nearestpointsthreadpool.h"

using namespace Face::FaceData;

void FaceAlignerIcp::CVTemplateMatchingSettings::serialize(cv::FileStorage &storage) const
{
    storage << "comparisonMethod" << comparisonMethod;
    storage << "inputImageType" << inputImageType;
    storage << "templateImage" << templateImage;
    storage << "center" << center;
}

void FaceAlignerIcp::CVTemplateMatchingSettings::deserialize(cv::FileStorage &storage)
{
    if (!storage.isOpened())
    {
        throw FACELIB_EXCEPTION("OpenCV storage is not opened");
    }

    storage["comparisonMethod"] >> comparisonMethod;
    int type;
    storage["inputImageType"] >> type;
    inputImageType = (InputImageType)type;
    storage["templateImage"] >> templateImage;
    std::vector<int> tmp;
    storage["center"] >> tmp;
    center = tmp.size() != 2 ? cv::Point() : cv::Point(tmp[0], tmp[1]);
    //storage["center"] >> center;
}

FaceAlignerIcp::FaceAlignerIcp(const Mesh &referenceFace, const std::string &templateMatchingFilePath) :
	threadPool(0), referenceFace(referenceFace)
{
    if (!templateMatchingFilePath.empty())
    {
        cv::FileStorage storage(templateMatchingFilePath, cv::FileStorage::READ);
        cvTemplateMatchingSettings.deserialize(storage);
    }
}

FaceAlignerIcp::~FaceAlignerIcp()
{
    if (threadPool) delete threadPool;
}

void FaceAlignerIcp::alignCentralize(Mesh &face) const
{
    face.centralize();
}

void FaceAlignerIcp::alignMaxZ(Mesh &face) const
{
    double maxZ = -1e300;
    double index = 0;
    face.centralize();
    for (int r = 0; r < face.pointsMat.rows; r++)
    {
        double z = face.pointsMat(r, 2);
        if (z > maxZ)
        {
            index = 0;
            maxZ = z;
        }
    }

    cv::Point3d translate(face.pointsMat(index, 0), face.pointsMat(index, 1), face.pointsMat(index, 2));
    face.translate(-translate);
}

/*void FaceAligner::alignNoseTip(Mesh &face) const
{
    LandmarkDetector lmDetector(face);
    Landmarks lm = lmDetector.detect();
    if (lm.is(Landmarks::Nosetip))
    {
        face.translate(-lm.get(Landmarks::Nosetip));
    }
    else
    {
        alignMaxZ(face);
    }
}*/

void FaceAlignerIcp::alignTemplateMatching(Mesh &face) const
{
    MapConverter refConverter;
    Map refDepth = SurfaceProcessor::depthmap(referenceFace, refConverter, 1.0, SurfaceProcessor::ZCoord);

    MapConverter inputConverter;
    //double padding = 30;
    Map depth = SurfaceProcessor::depthmap(face, inputConverter, /*cv::Point2d(referenceFace.minx - padding, referenceFace.miny - padding),
                                           cv::Point2d(referenceFace.maxx + padding, referenceFace.maxy + padding),*/ 1.0, SurfaceProcessor::ZCoord);

    int n = refDepth.h * refDepth.w;
    double refValuesShift = cv::sum(refDepth.values)[0] / n;
    Matrix refValues = refDepth.values - refValuesShift;

    int miny = -1;
    int minx = -1;
    double minSum = 1e300;
    //double minShift;
    for (int y = 0; y < depth.h - refDepth.h; y+=5)
    {
        for (int x = 0; x < depth.w - refDepth.w; x+=5)
        {
            Matrix tmp = depth.values(cv::Rect(x, y, refDepth.w, refDepth.h)).clone();
            double shift = cv::sum(tmp)[0] / n;
            tmp = tmp - shift;

            Matrix diff;
            cv::absdiff(tmp, refValues, diff);
            double s = cv::sum(diff)[0] / n;

            //qDebug() << s;

            if (s < minSum)
            {
                minSum = s;
                //minShift = shift;
                minx = x;
                miny = y;
            }

            /*tmp = depth.values.clone();
            double min, max; cv::minMaxIdx(tmp, &min, &max); tmp = (tmp-min)/(max-min);
            double realx = x - referenceFace.minx;
            double realy = y + referenceFace.maxy;
            cv::circle(tmp, cv::Point(realx, realy), 2, 0);
            cv::imshow("pre-align", tmp);
            qDebug() << s;
            cv::waitKey();*/
        }
    }

    if ((minx >= 0) && (miny >= 0))
    {
        //qDebug() << "Nosetip located at" << minx << miny << minSum;

        double realx = minx - referenceFace.minx;
        double realy = miny + referenceFace.maxy;

        /*Matrix tmp = depth.values.clone();
        double min, max; cv::minMaxIdx(tmp, &min, &max); tmp = (tmp-min)/(max-min);
        cv::circle(tmp, cv::Point(realx, realy), 2, 0);
        cv::imshow("pre-align", tmp);
        cv::waitKey();*/

        cv::Point3d p = inputConverter.MapToMeshCoords(depth, cv::Point2d(realx, realy));
        face.translate(-p);
    }
    else
    {
        std::cerr << "NOT FOUND!!!" << std::endl;
        alignCentralize(face);
    }
}

void FaceAlignerIcp::alignCVTemplateMatching(Mesh &face) const
{
    double min, max;
    //Matrix input;
    cv::Mat_<float> input;
    MapConverter converter;
    Map depthmap = SurfaceProcessor::depthmap(face, converter, 1.0, SurfaceProcessor::ZCoord);
    if (cvTemplateMatchingSettings.inputImageType == CVTemplateMatchingSettings::Texture)
    {
        MapConverter mc;
        input = SurfaceProcessor::depthmap(face, mc, 1.0, SurfaceProcessor::Texture_I).toMatrix(0, 0, 255);
    }
    else
    {

        if (cvTemplateMatchingSettings.inputImageType == CVTemplateMatchingSettings::Depth)
        {
            //depthmap.minMax(min, max);
            input = depthmap.values; // (depthmap.toMatrix() * (max - min)) + min;
        }
        else
        {
            depthmap.applyCvGaussBlur(7, 3);
            bool pcl = cvTemplateMatchingSettings.inputImageType == CVTemplateMatchingSettings::Eigen;
            CurvatureStruct cs = SurfaceProcessor::calculateCurvatures(depthmap, pcl);

            if (cvTemplateMatchingSettings.inputImageType == CVTemplateMatchingSettings::Mean)
                input = cs.meanMatrix();
            else if (cvTemplateMatchingSettings.inputImageType == CVTemplateMatchingSettings::Gauss)
                input = cs.gaussMatrix();
            if (cvTemplateMatchingSettings.inputImageType == CVTemplateMatchingSettings::Index)
                input = cs.indexMatrix();
            if (cvTemplateMatchingSettings.inputImageType == CVTemplateMatchingSettings::Eigen)
                input = cs.pclMatrix();
        }
    }

    cv::Mat_<float> result;
    cv::Point minLoc, maxLoc;
    cv::matchTemplate(input, cvTemplateMatchingSettings.templateImage, result, cvTemplateMatchingSettings.comparisonMethod);
    cv::minMaxLoc(result, &min, &max, &minLoc, &maxLoc);

    cv::Point p;
    if (cvTemplateMatchingSettings.comparisonMethod == cv::TM_SQDIFF ||
        cvTemplateMatchingSettings.comparisonMethod == cv::TM_SQDIFF_NORMED)
    {
        p = cv::Point(minLoc.x + cvTemplateMatchingSettings.center.x, minLoc.y + cvTemplateMatchingSettings.center.y);
    }
    else
    {
        p = cv::Point(maxLoc.x + cvTemplateMatchingSettings.center.x, maxLoc.y + cvTemplateMatchingSettings.center.y);
    }
    face.translate(-converter.MapToMeshCoords(depthmap, cv::Point2d(p.x, p.y)));
}

VectorOfPoints getPointCloudFromMatrix(const Matrix &points)
{
    VectorOfPoints result;
    for (int r = 0; r < points.rows; r++)
    {
        result.push_back(cv::Point3d(points(r,0), points(r,1), points(r,2)));
    }
    return result;
}

void FaceAlignerIcp::preAlign(Mesh &face, PreAlignTransform preAlignTransform) const
{
    switch (preAlignTransform)
    {
    case None:
        break;
    case Centralize:
        alignCentralize(face);
        break;
    case MaxZ:
        alignMaxZ(face);
        break;
    /*case NoseTipDetection:
        alignNoseTip(face);
        break;*/
    case TemplateMatching:
        alignTemplateMatching(face);
        break;
    case CVTemplateMatching:
        alignCVTemplateMatching(face);
        break;
    }
}

void FaceAlignerIcp::setEnableThreadPool(bool enable)
{
    if (!enable && threadPool) delete threadPool;
    if (enable && !threadPool) threadPool = new NearestPointsThreadPool();
}

void FaceAlignerIcp::align(Mesh &face, int maxIterations, PreAlignTransform preAlignTransform) const
{
    preAlign(face, preAlignTransform);

    Face::LinAlg::Procrustes3DResult progress;
    cv::flann::Index index;
    cv::Mat features;
    face.trainPointIndex(index, features, cv::flann::KMeansIndexParams());

    Matrix referencePoints = Matrix(referenceFace.pointsMat.rows, referenceFace.pointsMat.cols);
    Matrix transformedReference = Matrix(referenceFace.pointsMat.rows, referenceFace.pointsMat.cols);
    Matrix pointsToTransform = Matrix(referenceFace.pointsMat.rows, referenceFace.pointsMat.cols);
    for (int iteration = 0; iteration < maxIterations; iteration++)
    {
        referenceFace.pointsMat.copyTo(referencePoints);
        referenceFace.pointsMat.copyTo(transformedReference);
        for (int i = progress.preTranslations.size() - 1; i >= 0; i--)
        {
            Face::LinAlg::Procrustes3D::translate(transformedReference, -progress.postTranslations[i]);
            Face::LinAlg::Procrustes3D::inverseTransform(transformedReference, progress.rotations[i]);
            Face::LinAlg::Procrustes3D::translate(transformedReference, -progress.preTranslations[i]);
        }

        if (threadPool)
        {
            threadPool->getNearestPoints(&face.pointsMat, &transformedReference, &index, &pointsToTransform);
        }
        else
        {
            face.getNearestPoints(transformedReference, index, pointsToTransform);
        }

        // translation
        cv::Point3d centralizeReferences = Face::LinAlg::Procrustes3D::centralizedTranslation(referencePoints);
        Face::LinAlg::Procrustes3D::translate(referencePoints, centralizeReferences);

        cv::Point3d centralizePointsToTransform = Face::LinAlg::Procrustes3D::centralizedTranslation(pointsToTransform);
        Face::LinAlg::Procrustes3D::translate(pointsToTransform, centralizePointsToTransform);
        face.translate(centralizePointsToTransform);
        progress.preTranslations.push_back(centralizePointsToTransform);

        // SVD rotation
        Matrix rotation = Face::LinAlg::Procrustes3D::getOptimalRotation(pointsToTransform, referencePoints);
        face.transform(rotation);
        progress.rotations.push_back(rotation);

        // post translation
        face.translate(-centralizeReferences);
        progress.postTranslations.push_back(-centralizeReferences);
    }
}

namespace
{
	void filterPoints(Face::FaceData::Landmarks::Points &in, Face::FaceData::Landmarks::Points &reference)
	{
		auto inCopy = in; in.clear();
		auto referenceCopy = reference; reference.clear();
		auto n = referenceCopy.size();
		for (decltype(n) i = 0; i < n; i++)
		{
			const auto &p = inCopy[i];
			if (p.x != 0 && p.y != 0 && p.z != 0)
			{
				in.push_back(p);
				reference.push_back(referenceCopy[i]);
			}
			else
			{
                //std::cout << "filter points: removing point " << i << std::endl;
			}
		}
	}
}

FaceAlignerLandmark::FaceAlignerLandmark(const Landmarks &referenceLandmarks) :
	referenceLandmarks(referenceLandmarks)
{

}

void FaceAlignerLandmark::align(Mesh &face, Landmarks &landmarks) const
{
	// Check that the number of landmarks match the reference point count
	if (referenceLandmarks.points.size() != landmarks.points.size()) {
		std::string msg = "Landmarks size mismatch: reference " + std::to_string(referenceLandmarks.points.size()) + "; input: " + std::to_string(landmarks.points.size());
		std::cerr << msg << std::endl;
		FACELIB_EXCEPTION(msg);
	}

	// Filter-out landmarks with zero coords
	auto referencePoints = referenceLandmarks.points;
	auto filteredLandmarks = landmarks.points;
	filterPoints(filteredLandmarks, referencePoints);

	// Move reference to [0,0,0]
	auto centralizeReference = Face::LinAlg::Procrustes3D::centralizedTranslation(referencePoints);
	Face::LinAlg::Procrustes3D::translate(referencePoints, centralizeReference);

	// Move landmarks to the reference
	auto translation = Face::LinAlg::Procrustes3D::getOptimalTranslation(filteredLandmarks, referencePoints);
	Face::LinAlg::Procrustes3D::translate(filteredLandmarks, translation);
	Face::LinAlg::Procrustes3D::translate(landmarks.points, translation);
	face.translate(translation);
	
	// Rotate landmarks
	auto rotation = Face::LinAlg::Procrustes3D::getOptimalRotation(filteredLandmarks, referencePoints);
	//Face::LinAlg::Procrustes3D::transform(filteredLandmarks, rotation);
	Face::LinAlg::Procrustes3D::transform(landmarks.points, rotation);
	face.transform(rotation);

	// Move landmarks to the point where the reference was
	Face::LinAlg::Procrustes3D::translate(landmarks.points, -centralizeReference);
	face.translate(-centralizeReference);
}
