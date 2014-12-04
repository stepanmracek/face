#include "occipital/occipitalsensor.h"

using namespace Face::Sensors::Occipital;

OccipitalSensor::OccipitalSensor()
{
    depthData = Matrix::zeros(Height, Width);
    irData = Matrix::zeros(Height, Width);

    if (openni::OpenNI::initialize() != openni::STATUS_OK)
        throw FACELIB_EXCEPTION("Can't initialize OpenNI");

    if (device.open(openni::ANY_DEVICE) != openni::STATUS_OK)
        throw FACELIB_EXCEPTION("can't open OpenNI device");

    openni::DeviceInfo devInfo = device.getDeviceInfo();
    std::cout << devInfo.getVendor() << " " << devInfo.getName() << " @ " << devInfo.getUri() << std::endl;

    if (!device.hasSensor(openni::SENSOR_IR))
            throw FACELIB_EXCEPTION("Device does not have depth sensor");

    configureIR();
    configureDepth();
}

void OccipitalSensor::configureDepth()
{
    const openni::SensorInfo *sensorDepthInfo = device.getSensorInfo(openni::SENSOR_DEPTH);
    const openni::Array<openni::VideoMode> &depthVideoModes = sensorDepthInfo->getSupportedVideoModes();
    int depthSelection = -1;
    for (int i = 0; i < depthVideoModes.getSize(); i++)
    {
        const openni::VideoMode &vm = depthVideoModes[i];
        if (vm.getResolutionX() == Width && vm.getResolutionY() == Height &&
                vm.getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_100_UM &&
                vm.getFps() == 30)
            depthSelection = i;
    }

    if (depthSelection == -1)
        throw FACELIB_EXCEPTION("sensor does not support desired depth videoMode");

    if (depthStream.create(device, openni::SENSOR_DEPTH) != openni::STATUS_OK)
        throw FACELIB_EXCEPTION("can't create depth VideoStream");

    if (depthStream.setVideoMode(depthVideoModes[depthSelection]) != openni::STATUS_OK)
        throw FACELIB_EXCEPTION("can't set Depth videoMode");

    if (depthStream.start() != openni::STATUS_OK)
        throw FACELIB_EXCEPTION("can't start depth VideoStream");
}

void OccipitalSensor::configureIR()
{
    const openni::SensorInfo *sensorIrInfo = device.getSensorInfo(openni::SENSOR_IR);
    const openni::Array<openni::VideoMode> &irVideoModes = sensorIrInfo->getSupportedVideoModes();
    int irSelection = -1;
    for (int i = 0; i < irVideoModes.getSize(); i++)
    {
        const openni::VideoMode &vm = irVideoModes[i];
        if (vm.getResolutionX() == Width && vm.getResolutionY() == Height &&
                vm.getPixelFormat() == openni::PIXEL_FORMAT_GRAY16 &&
                vm.getFps() == 30)
            irSelection = i;
    }

    if (irSelection == -1)
        throw FACELIB_EXCEPTION("sensor does not support desired IR videoMode");

    if (irStream.create(device, openni::SENSOR_IR) != openni::STATUS_OK)
        throw FACELIB_EXCEPTION("can't create IR VideoStream");

    if (irStream.addNewFrameListener(this) != openni::STATUS_OK)
        throw FACELIB_EXCEPTION("can't register listener");

    if (irStream.setVideoMode(irVideoModes[irSelection]) != openni::STATUS_OK)
        throw FACELIB_EXCEPTION("can't set IR videoMode");

    if (irStream.start() != openni::STATUS_OK)
        throw FACELIB_EXCEPTION("can't start IR VideoStream");
}

OccipitalSensor::~OccipitalSensor()
{
    std::cout << "Cleaning up..." << std::endl;

    std::cout << "Stoppimg streams..." << std::endl;
    irStream.stop();
    depthStream.stop();

    std::cout << "Closing device..." << std::endl;
    device.close();

    std::cout << "Shutting down OpenNI..." << std::endl;
    openni::OpenNI::shutdown();
}

void OccipitalSensor::onNewFrame(openni::VideoStream &stream)
{
    /*static long i;
    i++;
    qDebug() << i << "onNewFrame()";*/
    if (irStream.readFrame(&irFrame) != openni::STATUS_OK || depthStream.readFrame(&depthFrame) != openni::STATUS_OK)
    {
        std::cerr << "readFrame not OK " << stream.isValid() << std::endl;
        return;
    }

    const openni::Grayscale16Pixel* irData = (const openni::Grayscale16Pixel*)(irFrame.getData());
    const openni::DepthPixel* depthData = (const openni::DepthPixel*)(depthFrame.getData());
    for (int r = 0; r < Height; r++)
    {
        for (int c = 0; c < Width; c++)
        {
            this->irData(r, c) = irData[Width*r + c];
            this->depthData(r, c) = depthData[Width*r + c];
        }
    }
}

void OccipitalSensor::scan()
{
    while (cv::waitKey(30) < 0)
    {
        cv::imshow("ir", irData/255.0);
        cv::imshow("depth", depthData/30000.0);
    }

    int frames = 20;
    Matrix cumulativeDepth = Matrix::zeros(Height, Width);
    Matrix hitCounter = Matrix::zeros(Height, Width);
    for (int i = 0; i < frames; i++)
    {
        for (int r = 0; r < Height; r++)
        {
            for (int c = 0; c < Width; c++)
            {
                if (depthData(r, c) > 5000 && depthData(r, c) < 15000)
                {
                    cumulativeDepth(r, c) += depthData(r, c);
                    hitCounter(r, c)++;
                }
            }
        }
        cv::imshow("ir", irData/255.0);
        cv::imshow("depth", cumulativeDepth/30000.0/(i+1));
        cv::waitKey(30);
    }

    for (int r = 0; r < Height; r++)
    {
        for (int c = 0; c < Width; c++)
        {
            if (hitCounter(r, c) == 0) continue;
            cumulativeDepth(r, c) = cumulativeDepth(r, c) / hitCounter(r, c);
        }
    }

    Face::FaceData::VectorOfPoints pointcloud;
    Face::FaceData::Mesh::Colors colors;
    for (int r = 0; r < Height; r++)
    {
        for (int c = 0; c < Width; c++)
        {
            if (cumulativeDepth(r, c) == 0) continue;

            float x, y, z;
            openni::CoordinateConverter::convertDepthToWorld(depthStream, c, r, cumulativeDepth(r, c), &x, &y, &z);
            pointcloud.push_back(cv::Point3d(x/10, y/10, -z/10));

            uchar intensity = irData(r, c);
            colors.push_back(Face::FaceData::Mesh::Color(intensity, intensity, intensity));
        }
    }
    m = Face::FaceData::Mesh::fromPointcloud(pointcloud, true, true);
    m.colors = colors;

    depthStream.start();

    cv::destroyAllWindows();
}
