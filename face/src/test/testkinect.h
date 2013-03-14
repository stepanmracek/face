#ifndef TESTKINECT_H
#define TESTKINECT_H

#include <QApplication>
#include <iostream>

//#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>

#include "kinect/kinnect.h"
#include "facelib/map.h"
#include "facelib/mesh.h"
#include "facelib/glwidget.h"
#include "facelib/surfaceprocessor.h"
#include "facetrack/realtimetrack.h"

class TestKinect
{
public:
    static int testScan(int argc, char *argv[])
    {
        /*RealTimeTrack rtTrack;

        double depth[640*480];
        bool mask[640*480];
        uint8_t rgb[640*480*3];
        ImageGrayscale grayScaleImg;
        ImageBGR colorImage;

        bool iterate = true;
        char *testWinName = "Test Scan";
        cv::namedWindow(testWinName);
        while (iterate)
        {
            if (!Kinect::getRGB(rgb))
            {
                qDebug() << "Kinect RGB error";
                exit(1);
            }

            char key = cv::waitKey(1);
            if (key != -1)
            {
                break;
            }

            grayScaleImg = Kinect::RGBToGrayscale(rgb);
            std::vector<cv::Rect> faces = rtTrack.trackFace(grayScaleImg);
            if (faces.size() == 0)
            {
                cv::imshow(testWinName, grayScaleImg);
                //qDebug() << "no face detected";
                continue;
            }

            cv::Rect &face = faces[0];
            int maskIndex = 0;
            for (int r = 0; r < 480; r++)
            {
                for (int c = 0; c < 640; c++)
                {
                    mask[maskIndex] = face.contains(cv::Point(c, r));
                    maskIndex++;
                }
            }

            if (!Kinect::getDepth(depth, mask, 450, 800))
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

            cv::rectangle(grayScaleImg, face, cv::Scalar(255));
            cv::imshow(testWinName, grayScaleImg);
        }
        cv::destroyWindow(testWinName);


        if (!Kinect::getRGB(rgb))
        {
            qDebug() << "Kinect RGB error";
            exit(1);
        }
        grayScaleImg = Kinect::RGBToGrayscale(rgb);
        std::vector<cv::Rect> faces = rtTrack.trackFace(grayScaleImg);
        if (faces.size() == 0)
        {
            qDebug() << "No face detected";
            exit(1);
        }

        cv::Rect &face = faces[0];
        int maskIndex = 0;
        for (int r = 0; r < 480; r++)
        {
            for (int c = 0; c < 640; c++)
            {
                mask[maskIndex] = face.contains(cv::Point(c, r));
                maskIndex++;
            }
        }

        if (!Kinect::getDepth(depth, 20, mask, 450, 800))
        {
            qDebug() << "Kinect depth error";
            exit(1);
        }*/

        Mesh mesh = Kinect::scanFace();
        mesh.writeOBJ("test/kinect-face.obj", '.');

        QApplication app(argc, argv);
        GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&mesh);
        widget.show();

        return app.exec();
    }

    static int testFaceDetect(int argc, char *argv[])
    {
        RealTimeTrack rtTrack;

        double depth[640*480];
        uint8_t rgb[640*480*3];
        ImageGrayscale grayScaleImg;
        ImageBGR colorImage;

        int iterationCounter = 0;
        int resultMeshCounter = 0;
        while (iterationCounter < 20)
        {
            iterationCounter++;

            if (!Kinect::getRGB(rgb))
            {
                qDebug() << "Kinect RGB error";
                exit(1);
            }
            grayScaleImg = Kinect::RGBToGrayscale(rgb);
            //cv::equalizeHist(grayScaleImg, grayScaleImg);

            colorImage = Kinect::RGBToColorMatrix(rgb);

            std::vector<cv::Rect> faces = rtTrack.trackFace(grayScaleImg);
            if (faces.size() > 0)
            {
                qDebug() << "Detected" << iterationCounter;
                cv::Rect &rect = faces[0];

                bool mask[640*480];
                int maskIndex = 0;
                for (int r = 0; r < 480; r++)
                {
                    for (int c = 0; c < 640; c++)
                    {
                        mask[maskIndex] = rect.contains(cv::Point(c, r));
                        maskIndex++;
                    }
                }

                if (!Kinect::getDepth(depth, mask, 200, 1200))
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

                Mesh mesh = Kinect::createMesh(depth, rgb);
                mesh.centralize();
                QString filename("face-"+QString::number(resultMeshCounter)+".obj");
                mesh.writeOBJ(filename, ',');
                resultMeshCounter++;

                cv::rectangle(grayScaleImg, rect, cv::Scalar(255));
            }

            cv::imshow("face", grayScaleImg);
            cv::waitKey(5);
        }
    }

    /*static void testPCL()
    {
        SimpleOpenNIViewer v;
        v.run();
    }*/

    /*static int testPCLMerge(int argc, char *argv[])
    {
        //QApplication app(argc, argv);
        //GLWidget widget;
        //widget.setWindowTitle("GL Widget");

        QVector<Mesh> meshes(5);
        for (int i = 0; i < 5; i++)
        {
            meshes[i] = Kinect::scanFace();
            //meshes[i].move(cv::Point3d((i-2)*100, 0, 0));
            //widget.addFace(&meshes[i]);

            pcl::PointCloud<pcl::PointXYZRGBA> pointcloud;
            for (int j = 0; j < meshes[i].points.size(); j++)
            {
                cv::Point3d &p = meshes[i].points[j];
                cv::Vec3b &c = meshes[i].colors[j];
                pcl::PointXYZRGBA pclPoint;
                pclPoint.x = p.x;
                pclPoint.y = p.y;
                pclPoint.z = -p.z;
                pclPoint.r = c[2];
                pclPoint.g = c[1];
                pclPoint.b = c[0];
                pointcloud.push_back(pclPoint);
            }

            QString fileName = "test/pointcloud" + QString::number(i) + ".pcd";
            pcl::io::savePCDFileBinary(fileName.toStdString(), pointcloud);
        }

        //widget.show();
        //app.exec();
    }*/
};

#endif // TESTKINECT_H
