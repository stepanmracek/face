#include "faceCommon/objectdetection/tracker.h"

using namespace Face::ObjectDetection;

Tracker::Tracker(const std::string &path)
{
    if (!classifier.load(path))
        throw FACELIB_EXCEPTION("can't load classifier " + path);
    init();
}

void Tracker::init()
{
    lastDetect = false;
    consecutiveDetects = 0;
    consecutiveNonDetects = 0;
    initialRegion = cv::Rect();
    lastRegion = cv::Rect();
    moveDisplacementThreshold = 7;
    areaDisplacementThreshold = 0.2;
    bigDisplacement = false;
}

cv::Point Tracker::rectCenter(const cv::Rect &rect)
{
    return cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
}

cv::Rect Tracker::getBiggestArea(const std::vector<cv::Rect> &regions)
{
    if (regions.size() == 0) return cv::Rect();

    int biggest = INT_MIN;
    int biggestIndex = -1;
    for (size_t i = 0; i < regions.size(); ++i)
    {
        int area = regions[i].area();
        if (area > biggest)
        {
            biggest = area;
            biggestIndex = i;
        }
    }
    return regions[biggestIndex];
}

cv::Rect Tracker::detect(const ImageGrayscale &img)
{
    std::vector<cv::Rect> regions;
    classifier.detectMultiScale(img, regions);
    bigDisplacement = false;

    if (regions.size() == 0)
    {
        // Object not detected
        consecutiveDetects = 0;
        consecutiveNonDetects++;
        initialRegion = cv::Rect();
        lastRegion = cv::Rect();
        lastDetect = false;
    }
    else
    {
        lastRegion = getBiggestArea(regions);
        lastDetect = true;
        consecutiveDetects++;
        consecutiveNonDetects = 0;

        if (consecutiveDetects == 1)
        {
            // Detected first object in row
            initialRegion = lastRegion;
        }
        else
        {
            // Calculate difference between first and last detected object
            cv::Point centerFirst = rectCenter(initialRegion);
            cv::Point centerCurrent = rectCenter(lastRegion);
            cv::Point centerDiff = centerFirst - centerCurrent;

            if (abs(centerDiff.x) > moveDisplacementThreshold || abs(centerDiff.y) > moveDisplacementThreshold)
            {
                bigDisplacement = true;
            }

            double areaDiff = ((double)(initialRegion.area())) / lastRegion.area();
            if (fabs(1.0 - areaDiff) > areaDisplacementThreshold)
            {
                bigDisplacement = true;
            }

            if (bigDisplacement)
            {
                initialRegion = lastRegion;
            }
        }
    }

    return lastRegion;
}

