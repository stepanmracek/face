#include "kinectsensorplugin.h"

#include "kinect.h"
#include "linalg/vector.h"
#include "linalg/matrixconverter.h"
#include "linalg/histogram.h"

KinectSensorPlugin::KinectSensorPlugin(const QString &faceDetectorPath) :
    detector(faceDetectorPath),
    rotRect(cv::Point2f(320, 240), cv::Size2f(120, 160), 0),
    rect(320 - 60, 240 - 80, 120, 160)
{
    Matrix maskMatrix = Matrix::zeros(480, 640);
    cv::ellipse(maskMatrix, rotRect, 1.0, CV_FILLED);
    ellipsePointsCount = 0;
    memset(depthMask, 0, sizeof(bool)*640*480);
    for (int r = 240 - 80; r < 240 + 80; r++)
    {
        for (int c = 320 - 60; c < 320 + 60; c++)
        {
            if (maskMatrix(r, c))
            {
                depthMask[640*r+c] = true;
                ellipsePointsCount++;
            }
        }
    }
}

KinectSensorPlugin::PositionResponse KinectSensorPlugin::depthStats()
{
    int count = 0;
    double sum = 0.0;
    for (int i = 0; i < 640*480; i++)
    {
        if (depthBuffer[i])
        {
            count++;
            sum += depthBuffer[i];
        }
    }

    double ratio = (double)count/ellipsePointsCount;
    double mean = sum/count;

    //qDebug() << ratio << mean;
    if (ratio < 0.3) return NoData;
    if (mean > 820) return MoveCloser;
    if (mean < 780) return MoveFar;
    return Ok;

    //Kinect::depthToMatrix(depthBuffer, depth);
    //Vector depthVec = MatrixConverter::matrixToColumnVector(depth);
    //Histogram depthHistogram(depthVec.toQVector(), 3, true, 750.0, 1000.0, true);
    //Matrix histogramPlot = depthHistogram.plot();
    //cv::imshow("histogram", histogramPlot);
}

void KinectSensorPlugin::putText(ImageGrayscale &img, const char *text)
{
    cv::putText(img, text, cv::Point(64, 240), CV_FONT_HERSHEY_SIMPLEX, 1.0, 0, 5);
    cv::putText(img, text, cv::Point(64, 240), CV_FONT_HERSHEY_SIMPLEX, 1.0, 255, 2);
}

bool KinectSensorPlugin::position()
{
    ImageGrayscale img(480, 640);
    //Matrix depth(480, 640);

    int consecutiveFaceDetection = 0;
    while (1)
    {
        bool detected = false;
        Kinect::getRGB(rgbBuffer);
        Kinect::getDepth(depthBuffer, depthMask, 0.0, 1000.0);
        Kinect::RGBToGrayscale(rgbBuffer, img);
        PositionResponse response = depthStats();
        if (response == Ok)
        {
            ImageGrayscale roi = img(rect);
            std::vector<cv::Rect> faces = detector.detect(roi);
            if (faces.size() > 0)
            {
                detected = true;
                //cv::rectangle(roi, faces[0], 255);
            }
        }

        cv::flip(img, img, 1);
        cv::ellipse(img, rotRect, 0, 2, CV_AA);
        cv::ellipse(img, rotRect, 255, 1, CV_AA);
        //cv::line(img, cv::Point(0, 240), cv::Point(640, 240), 255);
        //cv::line(img, cv::Point(320, 0), cv::Point(320, 480), 255);
        //cv::rectangle(img, rect, 255);
        if (response == MoveCloser)
        {
            putText(img, "Move closer");
        }
        else if (response == MoveFar)
        {
            putText(img, "Move far");
        }
        if (detected)
        {
            putText(img, "Ok");
            consecutiveFaceDetection ++;
            qDebug() << "consecutiveFaceDetection:" << consecutiveFaceDetection;
            if (consecutiveFaceDetection > 5)
            {
                cv::destroyAllWindows();
                return true;
            }
        }
        else
        {
            consecutiveFaceDetection = 0;
        }

        cv::imshow("rgb", img);
        //cv::imshow("depth", (depth-750)/250);
        char key = cv::waitKey(30);
        if (key == 27) break;
    }

    cv::destroyAllWindows();
    return false;
}

Mesh *KinectSensorPlugin::scan()
{
    Kinect::getDepth(depthBuffer, 5, depthMask, 600.0, 1000.0);
    Kinect::getRGB(rgbBuffer);
    Mesh *result = Kinect::createMesh(depthBuffer, rgbBuffer);
    result->centralize();
    result->calculateTriangles();
    return result;
}
