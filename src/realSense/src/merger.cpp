#include "realSense/merger.h"

#include <Poco/ClassLibrary.h>

#include "mergerimpl.h"

using namespace Face::Sensors::RealSense;

Merger::Merger(const Settings &settings, Face::ObjectDetection::Detector::Ptr faceDetector)
{
	impl = new MergerImpl(settings, faceDetector);
}

Merger::~Merger()
{
	delete impl;
}

Face::Sensors::SensorData Merger::sensorData()
{
	SensorData d;

	d.processedScan.mesh = impl->mesh();
	d.processedScan.landmarks = impl->landmarks();
	d.processedScan.texture = impl->getColorFrame().clone();
	d.processedScan.faceRegion = impl->getInitialDetectPosition();
	
	return d;
}

Merger::State Merger::getState()
{
	return impl->getState();
}

void Merger::start()
{
	impl->setState(STATE_IDLE);
	output.state = STATE_IDLE;
}

void Merger::stop()
{
	impl->setState(STATE_OFF);
	output.state = STATE_OFF;
}

void Merger::doLoop()
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
