#include "realtimetrack.h"

#include <QDebug>



RealTimeTrack::RealTimeTrack()
{
    qDebug() << "Reading classifiers";
    qDebug() << "face detect:" << faceDetect.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml");
    qDebug() << "eye detect:"<< eyeDetect.load("/usr/share/opencv/haarcascades/haarcascade_eye.xml");
    qDebug() << "nose detect:"<< noseDetect.load("/usr/share/opencv/haarcascades/haarcascade_mcs_nose.xml");
    qDebug() << "..done";
}

std::vector<cv::Rect>  RealTimeTrack::trackFace(ImageGrayscale &img)
{
    std::vector<cv::Rect> result;
    faceDetect.detectMultiScale(img, result, 2.0);
    return result;
}

int RealTimeTrack::trackTest()
{
    cv::VideoCapture capture(0);
    if (!capture.isOpened())
        return -1;

    cv::namedWindow("cam");
    cv::Mat frame;
    cv::Mat grey;
    cv::Mat roi;
    cv::Scalar white(255);
    std::vector<cv::Rect> faces;
    std::vector<cv::Rect> eyes;
    std::vector<cv::Rect> noses;
    cv::Size imSize(320, 240);
    while (1)
    {
        capture >> frame;
        cv::resize(frame, frame, imSize);
        cv::cvtColor(frame, grey, CV_BGR2GRAY);
        cv::flip(grey, grey, 1);
        cv::equalizeHist(grey, grey);

        faces.clear();
        faceDetect.detectMultiScale(grey, faces, 2.0);

        for (int i = 0; i < faces.size(); i++)
        {
            cv::Rect &face = faces[i];
            cv::rectangle(grey, face, white);
            roi = grey(face);
            //cv::equalizeHist(roi, roi);

            eyes.clear();
            eyeDetect.detectMultiScale(roi, eyes, 1.1);
            for (int j = 0; j < eyes.size(); j++)
            {
                cv::Rect &r = eyes[j];
                cv::Point p1; p1.x = face.x + r.x; p1.y = face.y + r.y;
                cv::Point p2; p2.x = face.x + r.x + r.width; p2.y = face.y + r.y + r.height;
                cv::rectangle(grey, p1, p2, white);
            }

            noses.clear();
            noseDetect.detectMultiScale(roi, noses, 1.1);
            for (int j = 0; j < noses.size(); j++)
            {
                cv::Rect &r = noses[j];
                cv::Point p1; p1.x = face.x + r.x; p1.y = face.y + r.y;
                cv::Point p2; p2.x = face.x + r.x + r.width; p2.y = face.y + r.y + r.height;
                cv::rectangle(grey, p1, p2, white);
            }
        }

        cv::imshow("cam", grey);
        if(cv::waitKey(30) >= 0) break;
    }

    return 0;
}

