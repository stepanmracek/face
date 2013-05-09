#include "kinect.h"

#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_sync.h>

#include "facetrack/realtimetrack.h"

bool Kinect::getDepth(double *depth, bool *mask, double minDistance, double maxDistance)
{
    short *buffer;
    uint32_t ts;
    if (freenect_sync_get_depth((void**)(&buffer), &ts, 0, FREENECT_DEPTH_REGISTERED) != 0)
        return false;

    for (int i = 0; i < n; i++)
    {
        depth[i] = 0;

        double z =  buffer[i]; // 100.0 / (-0.00307110156374 * buffer[i] + beta);
        if (z == 0) continue;
        if (mask != NULL && !mask[i]) continue;
        if (z > maxDistance) continue;
        if (z < minDistance) continue;

        depth[i] = z;
    }

    return true;
}

bool Kinect::getDepth(double *depth, int scansCount, bool *mask, double minDistance, double maxDistance)
{
    int usages[n];
    for (int i = 0; i < n; i++)
    {
        usages[i] = 0;
        depth[i] = 0;
    }

    for (int s = 0; s < scansCount; s++)
    {

        short *buffer;
        uint32_t ts;
        if (freenect_sync_get_depth((void**)(&buffer), &ts, 0, FREENECT_DEPTH_REGISTERED) != 0)
            return false;

        for (int i = 0; i < n; i++)
        {
            double z =  buffer[i];
            if (z == 0) continue;
            if (mask != NULL && !mask[i]) continue;
            if (z > maxDistance) continue;
            if (z < minDistance) continue;

            usages[i] += 1;
            depth[i] += z;
        }
    }

    for (int i = 0; i < n; i++)
    {
        if (usages[i] > 0)
        {
            depth[i] /= usages[i];
        }
    }

    return true;
}

bool Kinect::getRGB(uint8_t *rgb)
{
    uint8_t *buffer;
    uint32_t ts;
    if (freenect_sync_get_video((void**)&buffer, &ts, 0, FREENECT_VIDEO_RGB) != 0)
        return false;

    for (int i = 0; i < n*3; i++)
    {
        rgb[i] = buffer[i];
    }

    return true;
}

bool Kinect::getRGBIter(uint8_t *rgb, int scansCount)
{
    uint8_t *buffer;
    uint32_t ts;

    double *cumulativeBuffer = new double[n*3];
    for (int i = 0; i < n*3; i++)
    {
        cumulativeBuffer[i] = 0;
    }

    for (int iteration = 0; iteration < scansCount; iteration++)
    {
        if (freenect_sync_get_video((void**)&buffer, &ts, 0, FREENECT_VIDEO_RGB) != 0)
            return false;

        for (int i = 0; i < n*3; i++)
        {
            cumulativeBuffer[i] += buffer[i];
        }
    }

    for (int i = 0; i < n*3; i++)
    {
        rgb[i] = cumulativeBuffer[i]/scansCount;
    }

    delete [] cumulativeBuffer;
    return true;
}

VectorOfPoints Kinect::depthToVectorOfPoints(double *depth)
{
    VectorOfPoints result;

    int i = 0;
    for (int r = 0; r < 480; r++)
    {
        for (int c = 0; c < 640; c++)
        {
            if (depth[i] != 0)
            {
                double z =  depth[i];
                double coef = (z + mindistance) * scaleFactor;
                double y = -(r - 240) * coef;
                double x = -(320 - c) * coef;

                z = -z;

                cv::Point3d p(x, y, z);
                result << p;
            }

            i++;
        }
    }

    return result;
}

Mesh Kinect::createMesh(double *depth, uchar *rgb)
{
    int DepthIndextoPointIndex[n];

    Mesh result;
    int i = 0;
    for (int r = 0; r < 480; r++)
    {
        for (int c = 0; c < 640; c++)
        {
            if (depth[i] != 0)
            {
                double z =  -depth[i];
                double coef = (depth[i] + mindistance) * scaleFactor;
                double y = -(r - 240) * coef;
                double x = -(320 - c) * coef;

                // 3d data
                cv::Point3d p(x, y, z);
                result.points << p;
                //std::cout << p.z << std::endl;

                DepthIndextoPointIndex[i] = result.points.size()-1;

                // texture
                int i3 = 3*i;
                unsigned char r = rgb[i3];
                unsigned char g = rgb[i3 + 1];
                unsigned char b = rgb[i3 + 2];
                Color c(b, g, r);
                result.colors << c;
            }

            i++;
        }
    }

    result.recalculateMinMax();

    for (int i = 0; i < n; i++)
    {
        int x = i % 640;
        if (x == 639) continue;

        int y = i / 640;
        if (y == 479) continue;

        short val = depth[i];
        if (val == 0) continue;

        short down = depth[i+640];
        short right = depth[i+1];
        short downRight = depth[i+641];

        if (down != 0 && downRight != 0)
            result.triangles << cv::Vec3i(DepthIndextoPointIndex[i],DepthIndextoPointIndex[i+640],DepthIndextoPointIndex[i+641]);
        if (right != 0 && downRight != 0)
            result.triangles << cv::Vec3i(DepthIndextoPointIndex[i],DepthIndextoPointIndex[i+641],DepthIndextoPointIndex[i+1]);
    }

    return result;
}

ImageBGR Kinect::RGBToColorMatrix(uint8_t *rgb)
{
    ImageBGR result(480, 640);
    int i = 0;
    for (int r = 0; r < 480; r++)
    {
        for (int c = 0; c < 640; c++)
        {
            int i3 = 3*i;
            result(r, c) = Color(rgb[i3 + 2], rgb[i3 + 1], rgb[i3]);
            i++;
        }
    }
    return result;
}

ImageGrayscale Kinect::RGBToGrayscale(uint8_t *rgb)
{
    ImageGrayscale result(480, 640);
    int i = 0;
    for (int y = 0; y < 480; y++)
    {
        for (int x = 0; x < 640; x++)
        {
            int i3 = 3*i;
            double r = rgb[i3 + 2];
            double g = rgb[i3 + 1];
            double b = rgb[i3];
            //if (x == 0 && y == 0) qDebug() << r;
            result(y, x) = cv::saturate_cast<uint8_t>(0.299*r + 0.587*g + 0.114*b);

            i++;
        }
    }
    return result;
}

Mesh Kinect::scanFace(int scanIterations)
{
    RealTimeTrack rtTrack;
    int minDistanceFromSensor = 200;
    int maxDistanceFromSensor = 800;

    double depth[640*480];
    bool mask[640*480];
    uint8_t rgb[640*480*3];
    ImageGrayscale grayScaleImg;

    bool iterate = true;
    const char *testWinName = "Test Scan";
    cv::namedWindow(testWinName);
    while (iterate)
    {
        if (!Kinect::getRGB(rgb))
        {
            qDebug() << "Kinect RGB error";
            exit(1);
        }

        grayScaleImg = Kinect::RGBToGrayscale(rgb);
        std::vector<cv::Rect> faces = rtTrack.trackFace(grayScaleImg);

        int faceCount = faces.size();
        for (int i = 0; i < faceCount; i++)
        {
            int maskIndex = 0;
            for (int r = 0; r < 480; r++)
            {
                for (int c = 0; c < 640; c++)
                {
                    mask[maskIndex] = faces[i].contains(cv::Point(c, r));
                    maskIndex++;
                }
            }
        }
        if (faceCount > 0)
        {
            if (!Kinect::getDepth(depth, mask, minDistanceFromSensor, maxDistanceFromSensor))
            {
                qDebug() << "Kinect depth error";
                exit(1);
            }

            int depthIndex = 0;
            for (int r = 0; r < 480; r++)
            {
                for (int c = 0; c < 640; c++)
                {
                    if (depth[depthIndex] == 0)
                        grayScaleImg(r,c) = 0;
                    depthIndex++;
                }
            }

            for (int i = 0; i < faceCount; i++)
            {
                cv::rectangle(grayScaleImg, faces[i], cv::Scalar(255));
            }
        }
        cv::imshow(testWinName, grayScaleImg);

        char key = cv::waitKey(1);
        if (key != -1)
        {
            break;
        }
    }
    cv::destroyWindow(testWinName);

    Kinect::getDepth(depth, scanIterations, mask, minDistanceFromSensor, maxDistanceFromSensor);
    Kinect::getRGBIter(rgb, scanIterations);

    Mesh mesh = Kinect::createMesh(depth, rgb);
    mesh.centralize();
    mesh.calculateTriangles();
    return mesh;
}
