#include "softkinetic/guidrawer.h"

using namespace Face::Sensors::SoftKinetic;

void GUIDrawer::draw(const IDS325Sensor::Output &output)
{
    int colorWidth = 640;
    int colorHeight = 480;
    cv::Scalar white(255); //, 255, 255);
    cv::Scalar green(127); //64, 192, 64);
    ImageGrayscale gui = ImageGrayscale::zeros(colorHeight, colorWidth);

    int posX2 = colorWidth / 2 - 60;
    int posY2 = colorHeight * 2 / 3 - 20;
    int posY3 = colorHeight * 2 / 3 + 30;

    if (output.state == IDS325Sensor::Positioning || output.state == IDS325Sensor::Capturing)
    {
        output.currentFrame.copyTo(gui);
        cv::rectangle(gui, output.faceRegion, white, 1);
    }
    cv::flip(gui, gui, 1);

    cv::putText(gui, stateToText(output.state), cv::Point(10, gui.rows - 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, white, 1, CV_AA);

    switch (output.state)
    {
    case IDS325Sensor::Waiting:
    {
        /*cv::putText(gui, "detects: " + std::to_string(trackers.at(serials[0]).getConsecutiveDetects()) + "/" +
                std::to_string(settings.consecutiveDetectsToStartPositioning) + "; area: " +
                std::to_string(trackers.at(serials[0]).getLastRegion().area()) + "/" +
                std::to_string(settings.minimalFaceAreaToStartPositioning),
                cv::Point(faceDetectionRoi.width + 20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, white, 1, CV_AA);

        cv::putText(gui, "detects: " + std::to_string(trackers.at(serials[1]).getConsecutiveDetects()) + "/" +
                std::to_string(settings.consecutiveDetectsToStartPositioning) + "; area: " +
                std::to_string(trackers.at(serials[1]).getLastRegion().area()) + "/" +
                std::to_string(settings.minimalFaceAreaToStartPositioning),
                cv::Point(faceDetectionRoi.width + 20, faceDetectionRoi.height + 60), cv::FONT_HERSHEY_SIMPLEX,
                0.5, white, 1, CV_AA);*/
        break;
    }
    case IDS325Sensor::Positioning:
    {
        cv::putText(gui, positioningOutputToText(output.positioningOutput), cv::Point(posX2, posY2), cv::FONT_HERSHEY_SIMPLEX, 1.0, white, 1, CV_AA);
        if (output.positioningOutput == IDS325Sensor::DontMove)
        {
            double r = output.positioningProgress / 100.0;
            cv::rectangle(gui, cv::Rect(posX2, posY3, 200, 50), white, 1);
            cv::rectangle(gui, cv::Rect(posX2 + 2, posY3 + 2, 200 * r - 4, 50 - 4), green, -1);
        }
        break;
    }
    case IDS325Sensor::Capturing:
    {
        double r = output.capturingProgress / 100.0;
        cv::rectangle(gui, cv::Rect(posX2, posY3, 200, 50), white, 1);
        cv::rectangle(gui, cv::Rect(posX2 + 2, posY3 + 2, 200 * r - 4, 50 - 4), green, -1);
        break;
    }
    }

    if (output.state != IDS325Sensor::Off) cv::imshow("ds325", gui);
    else cv::destroyWindow("ds325");
}

std::string GUIDrawer::stateToText(IDS325Sensor::State state)
{
    switch (state)
    {
    case IDS325Sensor::Waiting:
        return "Waiting";
    case IDS325Sensor::Positioning:
        return "Positioning";
    case IDS325Sensor::Capturing:
        return "Capturing";
    }
    return "";
}

std::string GUIDrawer::positioningOutputToText(IDS325Sensor::PositioningOutput positioningOutput)
{
    switch (positioningOutput)
    {
    case IDS325Sensor::MoveCloser:
        return "Move Closer";
    case IDS325Sensor::MoveFar:
        return "Move Far";
    case IDS325Sensor::DontMove:
        return "Don't move";
    case IDS325Sensor::LookUp:
        return "Look Up";
    case IDS325Sensor::LookDown:
        return "Look Down";
    }
    return "";
}
