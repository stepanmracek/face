#include <iostream>
#include <stdio.h>
#include <vector>
#include <exception>

#include <DepthSense.hxx>

#include <opencv2/opencv.hpp>

#include "facelib/mesh.h"
#include "facelib/glwidget.h"
#include "facelib/surfaceprocessor.h"
#include "facetrack/realtimetrack.h"
#include "facelib/facealigner.h"

class DepthSensor
{
public:
    Mesh mesh;

    enum PositioningOutput { NoData, MoveCloser, MoveFar, DontMove };
    PositioningOutput positioningOutput;

private:

    enum SensorState { Off, Waiting, Positioning, Capturing };

    SensorState state;

    DepthSense::Context context;
    DepthSense::DepthNode depthNode;
    DepthSense::ColorNode colorNode;
    RealTimeTracker tracker;

    std::vector<DepthSense::FPVertex> vertices;
    std::vector<int> vertCounter;
    cv::Mat_<double> depth;
    cv::Mat_<cv::Vec3b> color;
    cv::Mat_<uchar> grayscale;
    cv::Mat_<uchar> gsResized;
    cv::Rect faceRegion;

    const static int ColorWidth = 640;
    const static int ColorHeight = 480;
    const static int DepthWidth = 320;
    const static int DepthHeight = 240;

    int desiredConfidence;
    bool deviceFound;
    int consecutiveFaceDetects;
    int consecutiveNoFaceDetects;
    int faceDetectsToStartPositioning;
    int noFaceDetectsToStopPositioning;
    float optimalDistanceMin;
    float optimalDistanceMax;
    int consecutiveOptimalDistance;
    int consecutiveOptimalDistanceToStartCapturing;
    int desiredCaptureFrames;
    int currentCaptureFrames;

    void setState(SensorState newState)
    {
        state = newState;

        switch(newState)
        {
        case Waiting:
            context.startNodes();
            context.run();

            consecutiveFaceDetects = 0;
            consecutiveNoFaceDetects = 0;
            break;
        case Positioning:
            consecutiveOptimalDistance = 0;
            break;
        case Capturing:
            currentCaptureFrames = 0;
            vertices = std::vector<DepthSense::FPVertex>(DepthWidth*DepthHeight, DepthSense::FPVertex(0.0, 0.0, 0.0));
            vertCounter = std::vector<int>(DepthWidth*DepthHeight, 0);
            break;
        case Off:
            context.quit();
            context.stopNodes();
        }
    }

    /**
     * New color sample event handler
     */
    void onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data)
    {
        int i = 0;
        for (int y = 0; y < ColorHeight; y++)
        {
            for (int x = 0; x < ColorWidth; x++)
            {
                color(y, x)[0] = data.colorMap[3*i];
                color(y, x)[1] = data.colorMap[3*i+1];
                color(y, x)[2] = data.colorMap[3*i+2];
                i++;
            }
        }
        cv::cvtColor(color, grayscale, CV_BGR2GRAY);
    }

    void detectFaces()
    {
        cv::resize(grayscale, gsResized, cv::Size(ColorWidth/4, ColorHeight/4));
        std::vector<cv::Rect> faces = tracker.detect(gsResized);
        if (faces.size() > 0)
        {
            faceRegion = faces[0];
            consecutiveFaceDetects++;
            consecutiveNoFaceDetects = 0;
        }
        else
        {
            faceRegion = cv::Rect();
            consecutiveFaceDetects = 0;
            consecutiveNoFaceDetects++;
        }
    }

    void onWaiting(DepthSense::DepthNode &node, DepthSense::DepthNode::NewSampleReceivedData &data)
    {
        detectFaces();

        if (consecutiveFaceDetects == faceDetectsToStartPositioning)
        {
            setState(Positioning);
        }
    }

    void distanceOfPointsWithinFaceRegion(DepthSense::DepthNode::NewSampleReceivedData &data)
    {
        float sum = 0.0;
        int count = 0;
        int n = DepthWidth*DepthHeight;
        for (int i = 0; i < n; i++)
        {
            const DepthSense::UV & uv = data.uvMap[i];
            if (uv.u == -FLT_MAX || uv.v == -FLT_MAX || data.depthMapFloatingPoint[i] == -2.0 || data.confidenceMap[i] < desiredConfidence) continue;
            int x = uv.u*ColorWidth/4;
            int y = uv.v*ColorHeight/4;
            if (faceRegion.contains(cv::Point(x, y)))
            {
                //gsResized(cv::Point(x,y)) = 255;
                sum += data.depthMapFloatingPoint[i];
                count++;
            }
        }

        float d = sum/count;
        if (d != d)
        {
            positioningOutput = NoData;
            consecutiveOptimalDistance = 0;
        }
        else if (d > optimalDistanceMax)
        {
            positioningOutput = MoveCloser;
            consecutiveOptimalDistance = 0;
        }
        else if (d < optimalDistanceMin)
        {
            positioningOutput = MoveFar;
            consecutiveOptimalDistance = 0;
        }
        else
        {
            positioningOutput = DontMove;
            consecutiveOptimalDistance++;
        }
    }

    void onPositioning(DepthSense::DepthNode &node, DepthSense::DepthNode::NewSampleReceivedData &data)
    {
        detectFaces();

        if (consecutiveNoFaceDetects == noFaceDetectsToStopPositioning)
        {
            setState(Waiting);
            return;
        }

        // calculate mean distance within faceRegion
        float sum = 0.0;
        int count = 0;
        int n = DepthWidth*DepthHeight;
        for (int i = 0; i < n; i++)
        {
            const DepthSense::UV & uv = data.uvMap[i];
            if (uv.u == -FLT_MAX || uv.v == -FLT_MAX || data.depthMapFloatingPoint[i] == -2.0 || data.confidenceMap[i] < desiredConfidence) continue;
            int x = uv.u*ColorWidth/4;
            int y = uv.v*ColorHeight/4;
            if (faceRegion.contains(cv::Point(x, y)))
            {
                gsResized(cv::Point(x,y))=255;
                sum += data.depthMapFloatingPoint[i];
                count++;
            }
        }

        distanceOfPointsWithinFaceRegion(data);
        if (consecutiveOptimalDistance == consecutiveOptimalDistanceToStartCapturing)
        {
            setState(Capturing);
        }
    }


    void onCapturing(DepthSense::DepthNode &node, DepthSense::DepthNode::NewSampleReceivedData &data)
    {
        //projectionHelper->get3DCoordinates();

        currentCaptureFrames++;
        int n = data.verticesFloatingPoint.size();
        for (int i = 0; i < n; i++)
        {
            const DepthSense::FPVertex &vertex = data.verticesFloatingPoint[i];
            if (vertex.z < 1 && data.confidenceMap[i] > desiredConfidence)
            {
                vertCounter[i]++;
                vertices[i].x += vertex.x;
                vertices[i].y += vertex.y;
                vertices[i].z += vertex.z;
            }
        }

        if (currentCaptureFrames >= desiredCaptureFrames)
        {
            currentCaptureFrames = 0;
            VectorOfPoints points;
            VectorOfColors colorsVec;
            for (int i = 0; i < n; i++)
            {
                int count = vertCounter[i];
                const DepthSense::UV & uv = data.uvMap[i];
                if (count == 0 || uv.u == -FLT_MAX || uv.v == -FLT_MAX ||
                    data.depthMapFloatingPoint[i] == -2.0 || data.confidenceMap[i] < desiredConfidence) continue;

                int x = uv.u*ColorWidth;
                int y = uv.v*ColorHeight;
                if (!faceRegion.contains(cv::Point(x/4, y/4))) continue;

                const DepthSense::FPVertex &vertex = vertices[i];
                points << cv::Point3d(vertex.x/count*1000, vertex.y/count*1000, -vertex.z/count*1000);
                colorsVec << color(y, x);
            }

            mesh = Mesh::fromPointcloud(points, true, true);
            mesh.colors = colorsVec;
            setState(Off);
        }
    }

    void drawGUI()
    {
        cv::Mat gui;
        cv::flip(grayscale, gui, 1);

        switch (state)
        {
        case Waiting:
            cv::putText(gui, "waiting", cv::Point(10, 450), cv::FONT_HERSHEY_SIMPLEX, 1.0, 255, 1, CV_AA);
            break;
        case Positioning:
            cv::putText(gui, "positioning", cv::Point(10, 450), cv::FONT_HERSHEY_SIMPLEX, 1.0, 255, 1, CV_AA);

            switch (positioningOutput)
            {
            case MoveCloser:
                cv::putText(gui, "Move forward", cv::Point(260, 300), cv::FONT_HERSHEY_SIMPLEX, 1.0, 255, 1, CV_AA);
                break;
            case MoveFar:
                cv::putText(gui, "Move backward", cv::Point(260, 300), cv::FONT_HERSHEY_SIMPLEX, 1.0, 255, 1, CV_AA);
                break;
            case DontMove:
                //ss << "Don't move " << consecutiveOptimalDistance << "/" << consecutiveOptimalDistanceToStartCapturing;
                cv::putText(gui, "Don't move", cv::Point(260, 300), cv::FONT_HERSHEY_SIMPLEX, 1.0, 255, 1, CV_AA);

                double r = (double)consecutiveOptimalDistance / consecutiveOptimalDistanceToStartCapturing;
                cv::rectangle(gui, cv::Rect(260, 350, 200, 50), 255, 1);
                cv::rectangle(gui, cv::Rect(260, 350, 200*r, 50), 255, -1);
                break;
            }

            break;
        case Capturing:
            //cv::putText(gui, "capturing", cv::Point(10, 450), cv::FONT_HERSHEY_SIMPLEX, 1.0, 255, 1, CV_AA);
            //ss << "Don't move " << currentCaptureFrames << "/" << desiredCaptureFrames;
            cv::putText(gui, "Capturing", cv::Point(260, 300), cv::FONT_HERSHEY_SIMPLEX, 1.0, 255, 1, CV_AA);

            double r = (double)currentCaptureFrames / desiredCaptureFrames;
            cv::rectangle(gui, cv::Rect(260, 350, 200, 50), 255, 1);
            cv::rectangle(gui, cv::Rect(260, 350, 200*r, 50), 255, -1);
            break;
        }

        cv::imshow("positioning demo", gui);
        cv::waitKey(1);
    }

    /**
     * New depth sample event handler
     */
    void onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
    {
        drawGUI();

        switch (state)
        {
        case Waiting:
            onWaiting(node, data);
            break;
        case Positioning:
            onPositioning(node, data);
            break;
        case Capturing:
            onCapturing(node, data);
            break;
        }


    }

    void configureDepthNode()
    {
        depthNode.newSampleReceivedEvent().connect(this, &DepthSensor::onNewDepthSample);

        DepthSense::DepthNode::Configuration config = depthNode.getConfiguration();
        config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
        config.framerate = 60;
        config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
        config.saturation = true;

        depthNode.setEnableDepthMapFloatingPoint(true);
        depthNode.setEnableVerticesFloatingPoint(true);
        depthNode.setEnableConfidenceMap(true);
        depthNode.setEnableUvMap(true);

        try
        {
            context.requestControl(depthNode,0);

            depthNode.setConfiguration(config);
        }
        catch (DepthSense::ArgumentException& e)
        {
            printf("Argument Exception: %s\n",e.what());
        }
        catch (DepthSense::UnauthorizedAccessException& e)
        {
            printf("Unauthorized Access Exception: %s\n",e.what());
        }
        catch (DepthSense::IOException& e)
        {
            printf("IO Exception: %s\n",e.what());
        }
        catch (DepthSense::InvalidOperationException& e)
        {
            printf("Invalid Operation Exception: %s\n",e.what());
        }
        catch (DepthSense::ConfigurationException& e)
        {
            printf("Configuration Exception: %s\n",e.what());
        }
        catch (DepthSense::StreamingException& e)
        {
            printf("Streaming Exception: %s\n",e.what());
        }
        catch (DepthSense::TimeoutException&)
        {
            printf("TimeoutException\n");
        }
    }

    void configureColorNode()
    {
        // connect new color sample handler
        colorNode.newSampleReceivedEvent().connect(this, &DepthSensor::onNewColorSample);

        DepthSense::ColorNode::Configuration config = colorNode.getConfiguration();
        config.frameFormat = DepthSense::FRAME_FORMAT_VGA;
        config.compression = DepthSense::COMPRESSION_TYPE_MJPEG;
        config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_50HZ;
        config.framerate = 25;

        colorNode.setEnableColorMap(true);

        try
        {
            context.requestControl(colorNode,0);

            colorNode.setConfiguration(config);
        }
        catch (DepthSense::ArgumentException& e)
        {
            printf("Argument Exception: %s\n",e.what());
        }
        catch (DepthSense::UnauthorizedAccessException& e)
        {
            printf("Unauthorized Access Exception: %s\n",e.what());
        }
        catch (DepthSense::IOException& e)
        {
            printf("IO Exception: %s\n",e.what());
        }
        catch (DepthSense::InvalidOperationException& e)
        {
            printf("Invalid Operation Exception: %s\n",e.what());
        }
        catch (DepthSense::ConfigurationException& e)
        {
            printf("Configuration Exception: %s\n",e.what());
        }
        catch (DepthSense::StreamingException& e)
        {
            printf("Streaming Exception: %s\n",e.what());
        }
        catch (DepthSense::TimeoutException&)
        {
            printf("TimeoutException\n");
        }
    }

    void configureNode(DepthSense::Node node)
    {
        if ((node.is<DepthSense::DepthNode>())&&(!depthNode.isSet()))
        {
            depthNode = node.as<DepthSense::DepthNode>();
            configureDepthNode();
            context.registerNode(node);
        }

        if ((node.is<DepthSense::ColorNode>())&&(!colorNode.isSet()))
        {
            colorNode = node.as<DepthSense::ColorNode>();
            configureColorNode();
            context.registerNode(node);
        }
    }

    void onNodeConnected(DepthSense::Device device, DepthSense::Device::NodeAddedData data)
    {
        configureNode(data.node);
    }

    void onNodeDisconnected(DepthSense::Device device, DepthSense::Device::NodeRemovedData data)
    {
        if (data.node.is<DepthSense::ColorNode>() && (data.node.as<DepthSense::ColorNode>() == colorNode))
            colorNode.unset();
        if (data.node.is<DepthSense::DepthNode>() && (data.node.as<DepthSense::DepthNode>() == depthNode))
            depthNode.unset();
    }

    void onDeviceConnected(DepthSense::Context context, DepthSense::Context::DeviceAddedData data)
    {
        if (!deviceFound)
        {
            data.device.nodeAddedEvent().connect(this, &DepthSensor::onNodeConnected);
            data.device.nodeRemovedEvent().connect(this, &DepthSensor::onNodeDisconnected);
            deviceFound = true;
        }
    }

    void onDeviceDisconnected(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data)
    {
        deviceFound = false;
    }

public:

    DepthSensor(const QString &haarFaceDetectorPath, int desiredConfidence = 200, int faceDetectsToStartPositioning = 20,
                int noFaceDetectsToStopPositioning = 60, float optimalDistanceMin = 0.25f, float optimalDistanceMax = 0.45f,
                int consecutiveOptimalDistanceToStartCapturing = 20, int desiredCaptureFrames = 20) :
        tracker(haarFaceDetectorPath),
        state(DepthSensor::Waiting),
        desiredConfidence(desiredConfidence),
        consecutiveFaceDetects(0),
        consecutiveNoFaceDetects(0),
        faceDetectsToStartPositioning(faceDetectsToStartPositioning),
        noFaceDetectsToStopPositioning(noFaceDetectsToStopPositioning),
        deviceFound(false),
        optimalDistanceMin(optimalDistanceMin),
        optimalDistanceMax(optimalDistanceMax),
        consecutiveOptimalDistanceToStartCapturing(consecutiveOptimalDistanceToStartCapturing),
        desiredCaptureFrames(desiredCaptureFrames),
        positioningOutput(DepthSensor::NoData)
    {
        depth = cv::Mat_<double>(DepthHeight, DepthWidth);
        color = cv::Mat_<cv::Vec3b>(ColorHeight, ColorWidth);

        context = DepthSense::Context::create("localhost");

        context.deviceAddedEvent().connect(this, &DepthSensor::onDeviceConnected);
        context.deviceRemovedEvent().connect(this, &DepthSensor::onDeviceDisconnected);

        // Get the list of currently connected devices
        std::vector<DepthSense::Device> devices = context.getDevices();

        // We are only interested in the first device
        if (devices.size() >= 1)
        {
            deviceFound = true;
            devices[0].nodeAddedEvent().connect(this, &DepthSensor::onNodeConnected);
            devices[0].nodeRemovedEvent().connect(this, &DepthSensor::onNodeDisconnected);

            std::vector<DepthSense::Node> nodes = devices[0].getNodes();
            for (int n = 0; n < (int)nodes.size();n++)
                configureNode(nodes[n]);
        }
    }

    ~DepthSensor()
    {
        if (colorNode.isSet()) context.unregisterNode(colorNode);
        if (depthNode.isSet()) context.unregisterNode(depthNode);
    }

    void scan()
    {
        setState(Waiting);
    }
};

/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
    FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj"));
    DepthSensor sensor("../../test/haar-face.xml");

    sensor.scan();
    Mesh m1 = sensor.mesh;
    SurfaceProcessor::smooth(m1, 0.1, 10);
    aligner.icpAlign(m1, 5, FaceAligner::NoseTipDetection);

    QApplication app(argc, argv);
    GLWidget w;
    w.addFace(&m1);
    w.show();

    QString filename = QInputDialog::getText(&w, "filename", "Filename:");
    if (!filename.isEmpty())
    {
        m1.writeBINZ(filename + ".binz");
        //sensor.writeRawData(filename + ".raw");
    }

    return app.exec();
}
