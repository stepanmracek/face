#include <iostream>
#include <fstream>
#include <Poco/ClassLibrary.h>

#include "pxcsensemanager.h"
#include "pxcmetadata.h"
#include "pxcsession.h"
#include "pxcprojection.h"
#include "pxccapture.h"
#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/surfaceprocessor.h"

#include "realSense/realsensesensor.h"
#include "realSense/merger.h"

int _main(int argc, char* argv[]) {
	pxcStatus sts;
	PXCSenseManager *senseManager = PXCSenseManager::CreateInstance();

	PXCVideoModule::DataDesc ddesc = {};
	ddesc.streams.depth.sizeMin.width = ddesc.streams.depth.sizeMax.width = 640;
	ddesc.streams.depth.sizeMin.height = ddesc.streams.depth.sizeMax.height = 480;
	ddesc.streams.depth.frameRate.min = 3;
	ddesc.streams.depth.frameRate.max = 30;

	ddesc.streams.color.sizeMin.width = ddesc.streams.color.sizeMax.width = 640;
	ddesc.streams.color.sizeMin.height = ddesc.streams.color.sizeMax.height = 480;
	ddesc.streams.color.frameRate.min = 3;
	ddesc.streams.color.frameRate.max = 30;

	sts = senseManager->EnableStreams(&ddesc);
	std::cout << "Enable streams status: " << sts << std::endl;

	sts = senseManager->Init();
	if (sts < PXC_STATUS_NO_ERROR) {
		std::cout << "Failed to locate any video stream(s); error: " << sts << std::endl;
		exit(sts);
	}

	//UtilRender renderc(L"Color");
	//UtilRender renderd(L"Depth");
	PXCPoint3DF32* vertices = new PXCPoint3DF32[640 * 480];
	PXCPointF32* uvmap = new PXCPointF32[640 * 480];

	PXCCapture::Device* device = senseManager->QueryCaptureManager()->QueryDevice();
	device->SetMirrorMode(PXCCapture::Device::MirrorMode::MIRROR_MODE_HORIZONTAL);
	PXCProjection* projection = device->CreateProjection();
	// TODO: device->SetIVCAMLaserPower(xxx)

	for (int nframes = 1; nframes <= 200; nframes++) {
		std::cout << "frame " << nframes << std::endl;

		sts = senseManager->AcquireFrame(true);

		if (sts < PXC_STATUS_NO_ERROR) {
			if (sts == PXC_STATUS_STREAM_CONFIG_CHANGED) {
				std::cout << "Stream configuration was changed, re-initializing" << std::endl;
				senseManager->Close();
			} else if (sts == PXC_STATUS_DEVICE_LOST) {
				std::cout << "Device lost: " << sts << std::endl;
			} else {
				std::cout << "Acquire frame error: " << sts << std::endl;
			}
			break;
		}

		const PXCCapture::Sample *sample = senseManager->QuerySample();
		if (sample) {
			if (sample->depth) {
				//renderd.RenderFrame(sample->depth);
				projection->QueryVertices(sample->depth, vertices);
				projection->QueryUVMap(sample->depth, uvmap);
			}
			else
				std::cout << "No depth" << std::endl;

			/*if (sample->color)
				renderc.RenderFrame(sample->color);
				else
				std::cout << "No color" << std::endl;*/
		}

		senseManager->ReleaseFrame();
	}

	Face::FaceData::VectorOfPoints points;
	for (int i = 0; i < 640 * 480; i++)
		points.push_back(cv::Point3d(vertices[i].x, vertices[i].y, -vertices[i].z));

	auto mesh = Face::FaceData::Mesh(); // ::fromSensorData(points, Face::FaceData::VectorOfPoints(), Face::FaceData::Mesh::Colors(), 640, 480, true);
	Face::FaceData::SurfaceProcessor::mdenoising(mesh, 0.04f, 10, 10);
	mesh.writeOBJ("test.obj");	

	std::cout << "Exiting" << std::endl;
	senseManager->Release();
	return 0;
}

POCO_BEGIN_MANIFEST(Face::Sensors::ISensor)
	POCO_EXPORT_CLASS(Face::Sensors::RealSense::Merger)
	POCO_EXPORT_CLASS(Face::Sensors::RealSense::RealSenseSensor)
POCO_END_MANIFEST