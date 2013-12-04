#include <iostream>
#include <stdio.h>
#include <vector>
#include <exception>

#include <DepthSense.hxx>

#include <opencv2/opencv.hpp>

#include <facelib/mesh.h>
#include <facelib/glwidget.h>
#include <facelib/surfaceprocessor.h>

class DepthSensor
{
public:
    Mesh mesh;
private:

    DepthSense::Context context;
    DepthSense::DepthNode depthNode;
    DepthSense::ColorNode colorNode;

    uint32_t colorFramesCount;
    uint32_t depthFramesCount;

    bool g_bDeviceFound;

    DepthSense::ProjectionHelper* projectionHelper;
    //DepthSense::StereoCameraParameters stereoCamParams;

    /**
     * New color sample event handler
     */
    void onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data)
    {
        if (!positioning) return;
        //std::cout << "Color " << (colorFramesCount++) << " " << data.colorMap.size() << std::endl;

        int i = 0;
        cv::Mat_<cv::Vec3b> color(480, 640);
        for (int y = 0; y < 480; y++)
        {
            for (int x = 0; x < 640; x++)
            {
                color(y, x)[0] = data.colorMap[3*i];
                color(y, x)[1] = data.colorMap[3*i+1];
                color(y, x)[2] = data.colorMap[3*i+2];
                i++;
            }
        }

        cv::imshow("color", color);
    }

    bool positioning;
    void onPositioning(DepthSense::DepthNode &node, DepthSense::DepthNode::NewSampleReceivedData &data)
    {
        cv::Mat_<double> depth(240, 320);
        int i = 0;
        for (int r = 0; r < 240; r++)
        {
            for (int c = 0; c < 320; c++)
            {
                double d = data.depthMapFloatingPoint[i++];
                if (d > 1) d = 1;
                if (d < 0) d = 0;
                depth(r, c) = d;
            }
        }
        depth = 1 - depth;
        cv::imshow("depth", depth);
        if ((char)cv::waitKey(1) == 27)
        {
            cv::destroyAllWindows();
            positioning = false;
            vertices = std::vector<DepthSense::FPVertex>(320*240, DepthSense::FPVertex(0.0, 0.0, 0.0));
            vertCounter = std::vector<int>(320*240, 0);
        }
    }

    std::vector<DepthSense::FPVertex> vertices;
    std::vector<int> vertCounter;
    void onCapturing(DepthSense::DepthNode &node, DepthSense::DepthNode::NewSampleReceivedData &data)
    {
        //projectionHelper->get3DCoordinates();

        static int frames = 0;
        frames++;
        int n = data.verticesFloatingPoint.size();
        for (int i = 0; i < n; i++)
        {
            const DepthSense::FPVertex &vertex = data.verticesFloatingPoint[i];
            if (vertex.z < 1 && data.confidenceMap[i] > 100)
            {
                vertCounter[i]++;
                vertices[i].x += vertex.x;
                vertices[i].y += vertex.y;
                vertices[i].z += vertex.z;
            }
        }

        if (frames >= 20)
        {
            VectorOfPoints points;

            for (int i = 0; i < n; i++)
            {
                const DepthSense::FPVertex &vertex = vertices[i];
                double count = vertCounter[i];
                if (count > 0.1)
                {
                    points << cv::Point3d(vertex.x/count*100, vertex.y/count*100, -vertex.z/count*100);
                }
            }
            mesh = Mesh::fromPointcloud(points, true, true);

            SurfaceProcessor::zsmooth(mesh, 0.5, 10);
            context.quit();
        }
    }


    /**
     * New depth sample event handler
     */
    void onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
    {
        positioning ? onPositioning(node, data) : onCapturing(node, data);
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
        printf("Node disconnected\n");
    }

    void onDeviceConnected(DepthSense::Context context, DepthSense::Context::DeviceAddedData data)
    {
        if (!g_bDeviceFound)
        {
            data.device.nodeAddedEvent().connect(this, &DepthSensor::onNodeConnected);
            data.device.nodeRemovedEvent().connect(this, &DepthSensor::onNodeDisconnected);
            g_bDeviceFound = true;
        }
    }

    void onDeviceDisconnected(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data)
    {
        g_bDeviceFound = false;
        printf("Device disconnected\n");
    }

public:

    DepthSensor()
    {
        positioning = true;
        colorFramesCount = 0;
        depthFramesCount = 0;
        g_bDeviceFound = false;
        //projectionHelper = NULL;

        context = DepthSense::Context::create("localhost");

        context.deviceAddedEvent().connect(this, &DepthSensor::onDeviceConnected);
        context.deviceRemovedEvent().connect(this, &DepthSensor::onDeviceDisconnected);

        // Get the list of currently connected devices
        std::vector<DepthSense::Device> devices = context.getDevices();

        // We are only interested in the first device
        if (devices.size() >= 1)
        {
            g_bDeviceFound = true;

            devices[0].nodeAddedEvent().connect(this, &DepthSensor::onNodeConnected);
            devices[0].nodeRemovedEvent().connect(this, &DepthSensor::onNodeDisconnected);

            std::vector<DepthSense::Node> nodes = devices[0].getNodes();

            printf("Found %lu nodes\n",nodes.size());

            for (int n = 0; n < (int)nodes.size();n++)
                configureNode(nodes[n]);
        }
    }

    void run()
    {
        context.startNodes();
        context.run();
        context.stopNodes();

        if (colorNode.isSet()) context.unregisterNode(colorNode);
        if (depthNode.isSet()) context.unregisterNode(depthNode);

        //if (projectionHelper)
        //    delete projectionHelper;
    }
};

/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
    DepthSensor sensor;
    sensor.run();

    QApplication app(argc, argv);
    GLWidget w;
    w.addFace(&sensor.mesh);
    w.show();

    return app.exec();
}
