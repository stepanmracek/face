#include "realSense/realsensesensor.h"

#include "realsensesensorimpl.h"
#include "realsensedetector.h"
#include "realsensedepthfacetracker.h"

#include <pxcsession.h>
#include <pxccapture.h>

#include <locale>
#include <sstream>

using namespace Face::Sensors::RealSense;

RealSenseSensor::RealSenseSensor(const Settings &settings, Face::ObjectDetection::Detector::Ptr faceDetector)
{
	impl = new RealSenseSensorImpl(settings, faceDetector);
}

RealSenseSensor::RealSenseSensor(const Settings &settings, FaceDetector faceDetector, const std::string& fdetectPath) {
	impl = new RealSenseSensorImpl(settings, faceDetector, fdetectPath);
}

RealSenseSensor::~RealSenseSensor()
{
	delete impl;
}

Face::Sensors::SensorData RealSenseSensor::sensorData()
{
	SensorData d;
	d.rawScan.mesh = impl->rawMesh();
	d.rawScan.landmarks = impl->rawLandmarks();
	d.rawScan.texture = impl->getColorFrame().clone();
	d.rawScan.faceRegion = impl->getInitialDetectPosition();

	d.processedScan.mesh = impl->averagedMesh();
	d.processedScan.landmarks = impl->averagedLandmarks();
	d.processedScan.texture = impl->getColorFrame().clone();
	d.processedScan.faceRegion = impl->getInitialDetectPosition();
	
	d.depthImage = impl->depth();

	/*d.texture = impl->getColorFrame();
	d.faceRegion = impl->getInitialDetectPosition();*/

	return d;
}

void RealSenseSensor::start()
{
	impl->setState(STATE_IDLE);
	output.state = STATE_IDLE;
}

void RealSenseSensor::stop()
{
	impl->setState(STATE_OFF);
	output.state = STATE_OFF;
}

void RealSenseSensor::doLoop()
{
	output.previousState = output.state;
	output.state = impl->go();

	output.capturingProgress = impl->getCapturingProgress();
	output.currentFrame = impl->getGrayscaleFrame();
	output.faceRegion = impl->getInitialDetectPosition();
	output.positioningOutput = impl->getPositioningOutput();
	output.positioningProgress = impl->getPositioningProgress();
	output.stopGesture = impl->isCoverGesture();
}

std::string toNarrow(const wchar_t *s, char dfault = '?', const std::locale& loc = std::locale()) {
	std::ostringstream stm;

	while (*s != L'\0') {
		stm << std::use_facet<std::ctype<wchar_t> >(loc).narrow(*s++, dfault);
	}
	return stm.str();
}

bool tryToNumber(const std::string& s, unsigned int& number) {
	std::string numString;
	for (auto i = 0; i < s.size() && i < 10; ++i) {
		auto c = s[i];
		if (isdigit(c)) {
			numString.push_back(c);
		}
	}

	return Poco::NumberParser::tryParseUnsigned(numString, number);
}

std::vector<unsigned int> RealSenseSensor::connectedSensors() {
	std::vector<unsigned int> sensors;

	PXCSession* session = PXCSession::CreateInstance();

	PXCSession::ImplDesc desc1 = { };
	desc1.group = PXCSession::IMPL_GROUP_SENSOR;
	desc1.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;

	for (int m = 0; ; m++) {
		PXCSession::ImplDesc desc2;
		if (session->QueryImpl(&desc1, m, &desc2) < PXC_STATUS_NO_ERROR) {
			break;
		}
		std::cout << "Module (" << m << "): " << desc2.friendlyName << std::endl;

		PXCCapture* capture = 0;
		if (session->CreateImpl<PXCCapture>(&desc2, &capture) < PXC_STATUS_NO_ERROR) {
			continue;
		}

		for (int d = 0; ; d++) {
			PXCCapture::DeviceInfo dinfo;
			if (capture->QueryDeviceInfo(d, &dinfo) < PXC_STATUS_NO_ERROR) {
				break;
			}
			std::cout << "Device (" << d << "): " << dinfo.name << std::endl;
			if (dinfo.model != PXCCapture::DEVICE_MODEL_F200 && dinfo.model != PXCCapture::DEVICE_MODEL_R200) {
				continue;
			}
			std::string serialStr = toNarrow(dinfo.serial);
			std::cout << "Found device type: " << std::hex << static_cast<int>(dinfo.model) << " with serial: " << serialStr << std::endl;
			unsigned int serial = 0;
			if (tryToNumber(serialStr, serial)) {
				sensors.push_back(serial);
			} else {
				std::cout << "Serial number " << serialStr << " could not be parsed." << std::endl;
			}
		}

		capture->Release();
	}

	session->Release();

	return sensors;
}
