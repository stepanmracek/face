#include "faceCommon/facedata/facefeaturesanotation.h"

#include <opencv/cv.h>
#include <Poco/Path.h>
#include <Poco/File.h>

#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/map.h"
#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceCommon/facedata/landmarks.h"
#include "faceCommon/linalg/kernelgenerator.h"
#include "faceCommon/linalg/loader.h"

using namespace Face::FaceData;

struct FaceFeaturesAnotationStruct
{
    //Matrix depth;
    //Matrix curvature;
    Matrix texture;
    std::vector<cv::Point2d> points;
    std::string windowName;
    //int mixture;
};

void FaceFeaturesAnotationShowFace(FaceFeaturesAnotationStruct &anotationStruct)
{
	// ((anotationStruct.mixture/10.0) * anotationStruct.depth + (1.0-anotationStruct.mixture/10.0)*anotationStruct.curvature);
	Matrix img = anotationStruct.texture.clone();

	for (const cv::Point2d &p : anotationStruct.points)
	{
		cv::circle(img, cv::Point(p.x, p.y), 2, cv::Scalar(0));
		cv::circle(img, cv::Point(p.x, p.y), 3, cv::Scalar(1));
	}

	std::vector<cv::Point2d> &pts = anotationStruct.points;
	if (anotationStruct.points.size() >= 4)
	{
		cv::line(img, pts[0], pts[1], 0);
		cv::line(img, pts[1], pts[2], 0);
		cv::line(img, pts[2], pts[3], 0);
	}
	if (anotationStruct.points.size() >= 9)
	{
        cv::line(img, pts[4], pts[5], 0);
		cv::line(img, pts[5], pts[6], 0);
		cv::line(img, pts[6], pts[7], 0);
		cv::line(img, pts[7], pts[8], 0);
    }
	if (anotationStruct.points.size() >= 15)
	{
		cv::line(img, pts[9], pts[10], 0);
		cv::line(img, pts[10], pts[11], 0);
		cv::line(img, pts[11], pts[12], 0);
		cv::line(img, pts[12], pts[13], 0);
		cv::line(img, pts[13], pts[14], 0);
		cv::line(img, pts[14], pts[9], 0);
	}
	if (anotationStruct.points.size() >= 21)
	{
		cv::line(img, pts[15], pts[16], 0);
		cv::line(img, pts[16], pts[17], 0);
		cv::line(img, pts[17], pts[18], 0);
		cv::line(img, pts[18], pts[19], 0);
		cv::line(img, pts[19], pts[20], 0);
		cv::line(img, pts[20], pts[15], 0);
	}

    cv::imshow(anotationStruct.windowName, img);
}

void FaceFeaturesAnotationTrackbarCallback(int /*pos*/, void* userdata)
{
    FaceFeaturesAnotationStruct &anotationStruct = *((FaceFeaturesAnotationStruct *)userdata);
    FaceFeaturesAnotationShowFace(anotationStruct);
}

void FaceFeaturesAnotationMouseCallback(int event, int x, int y, int /*flags*/, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        FaceFeaturesAnotationStruct &anotationStruct = *((FaceFeaturesAnotationStruct *)userdata);

        anotationStruct.points.push_back(cv::Point2d(x,y));
        FaceFeaturesAnotationShowFace(anotationStruct);
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        FaceFeaturesAnotationStruct &anotationStruct = *((FaceFeaturesAnotationStruct *)userdata);

        if (anotationStruct.points.size() > 0)
        {
            anotationStruct.points.erase(anotationStruct.points.end()-1);
            FaceFeaturesAnotationShowFace(anotationStruct);
        }
    }
}

Landmarks FaceFeaturesAnotation::anotate(Mesh &mesh, bool &success)
{
    std::string windowName = "face";
    MapConverter textureConverter;
	Map texture = SurfaceProcessor::depthmap(mesh, textureConverter, 2.0, SurfaceProcessor::ZCoord); // Texture_I);

    FaceFeaturesAnotationStruct anotationStruct;
    anotationStruct.texture = texture.toMatrix();
    anotationStruct.windowName = windowName;

    cv::namedWindow(windowName);
    cv::setMouseCallback(windowName, FaceFeaturesAnotationMouseCallback, &anotationStruct);

    FaceFeaturesAnotationShowFace(anotationStruct);
    cv::waitKey(0);
    cv::destroyWindow(windowName);

    Landmarks l;
    if (anotationStruct.points.size() == l.points.size())
    {
        MapConverter depthConverter;
        Map depth = SurfaceProcessor::depthmap(mesh, depthConverter, 2.0, SurfaceProcessor::ZCoord);

        success = true;
        for (unsigned int i = 0; i < l.points.size(); i++)
        {
            l.points[i] = depthConverter.MapToMeshCoords(depth, anotationStruct.points[i]);
        }
        //success = l.check();
    }
    else
    {
        success = false;
    }
    return l;
}

void FaceFeaturesAnotation::anotateFromFiles(const std::string &dirPath)
{
    std::vector<std::string> files = Face::LinAlg::Loader::listFiles(dirPath, "*.binz", Face::LinAlg::Loader::AbsoluteFull);

    for (const std::string &f : files)
    {
        std::cout << f << std::endl;

        Poco::Path path(f);
        std::string landmarksPath = dirPath + Poco::Path::separator() + path.getBaseName() + ".yml";
        if (Poco::File(landmarksPath).exists()) continue;

        Mesh mesh = Mesh::fromFile(f, false);
        bool success;
        Landmarks lm = anotate(mesh, success);
        std::cout << " Success: " << success << std::endl;
        if (success) lm.serialize(landmarksPath);

        char key = cv::waitKey();
        if (key == 27) break;
    }
}
