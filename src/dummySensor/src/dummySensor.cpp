#include "dummySensor.h"

#include <Poco/ClassLibrary.h>

#include "faceCommon/settings/settings.h"

using namespace Face::Sensors;

DummySensor::DummySensor()
{
    mesh = Face::FaceData::Mesh::fromFile(Face::Settings::instance().settingsMap[Face::Settings::MeanFaceModelPathKey]);
}

DummySensor::DummySensor(const std::string &meshPath)
{
    mesh = Face::FaceData::Mesh::fromFile(meshPath);
}

SensorData DummySensor::sensorData()
{
    SensorData d;
    d.rawScan.mesh = mesh;
    //d.processedScan.mesh = mesh;
    return d;
}

POCO_BEGIN_MANIFEST(Face::Sensors::ISensor)
	POCO_EXPORT_CLASS(Face::Sensors::DummySensor)
POCO_END_MANIFEST
