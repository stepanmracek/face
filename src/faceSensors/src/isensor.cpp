#include "faceSensors/isensor.h"
#include "Poco/Path.h"
#include "faceCommon/linalg/loader.h"
using namespace Face::Sensors;

void Scan::serialize(const std::string &path, bool obj) const
{
	if (obj)
	{
		size_t start = path.find_last_of(Poco::Path::separator()) + 1;
		auto basename = path.substr(start);
		this->mesh.writeOBJ(path + ".obj", basename);
	}
	else
		this->mesh.writeBINZ(path + ".binz");

	cv::imwrite(path + ".png", this->texture);
	this->landmarks.serialize(path + "-landmarks.yml");

	cv::FileStorage writeStorage(path + "-metadata.yml", cv::FileStorage::WRITE);
	writeStorage << "faceRegionW" << this->faceRegion.width;
	writeStorage << "faceRegionH" << this->faceRegion.height;
	writeStorage << "faceRegionX" << this->faceRegion.x;
	writeStorage << "faceRegionY" << this->faceRegion.y;
	/*writeStorage << "allPoints" << "[";
	/*for (const cv::Point3d &p : allPoints.points)
		writeStorage << p;
	writeStorage << "]";*/

	writeStorage.release();
}

Scan::Scan(const std::string &path)
{
	std::string basePath = path.substr(0, path.find_last_of('.'));

	this->mesh = Face::FaceData::Mesh::fromFile(path);
	this->texture = cv::imread(basePath + ".png");
	this->landmarks = Face::FaceData::Landmarks(basePath + "-landmarks.yml");

	cv::FileStorage readStorage(basePath + "-metadata.yml", cv::FileStorage::READ);
	readStorage["faceRegionW"] >> this->faceRegion.width;
	readStorage["faceRegionH"] >> this->faceRegion.height;
	readStorage["faceRegionX"] >> this->faceRegion.x;
	readStorage["faceRegionY"] >> this->faceRegion.y;
}

SensorLoader::SensorLoader(const std::string pluginDirectory) {
    auto libNames = Face::LinAlg::Loader::listFiles(pluginDirectory, "*F3D.sensor.*" + Poco::SharedLibrary::suffix(), Face::LinAlg::Loader::Filename);
    for (const auto &libName : libNames)
    {
        try
        {
            std::cout << "Loading " << libName << std::flush;
            loader.loadLibrary(libName);
            std::cout << ": OK" << std::endl;
        }
        catch(...)
        {
            std::cout << ": failed" << std::endl;
        }
    }

    for (SensorClassLoader::Iterator it = loader.begin(); it != loader.end(); ++it) {
        std::cout << "library path: " << it->first << std::endl;
        SensorManifest::Iterator itMan(it->second->begin());
        SensorManifest::Iterator endMan(it->second->end());
        for (; itMan != endMan; ++itMan) {
            if (loader.canCreate(itMan->name()))
            {
                std::cout << "  sensor name: " << itMan->name() << std::endl;
                sensorNames.push_back(itMan->name());
            }
        }
    }
}

ISensor::Ptr SensorLoader::getSensor(const std::string &sensorName)
{
    return loader.create(sensorName);
}

void ISensor::scan()
{
	start();

	auto white = cv::Scalar(255);
	auto black = cv::Scalar(0);
	
	while (output.state != STATE_OFF) {
		doLoop();

		ImageGrayscale img = output.currentFrame.clone();

		if (output.state == STATE_IDLE) {
			for (unsigned char *val = img.data; val != img.dataend; val++) {
				*val = *val / 2;
			}
		}

		if (output.stopGesture) {
			//cv::putText(img, "Sensor covered", cv::Point(120, 50), CV_FONT_HERSHEY_SIMPLEX, 1.0, white, 2, CV_AA);
			stop();
		}

		if (output.state == STATE_POSITIONING) {
			cv::putText(img, getPosOutputName(output.positioningOutput), cv::Point(122, 202), CV_FONT_HERSHEY_SIMPLEX, 1.0, black, 3, CV_AA);
			cv::putText(img, getPosOutputName(output.positioningOutput), cv::Point(120, 200), CV_FONT_HERSHEY_SIMPLEX, 1.0, white, 2, CV_AA);

			cv::Rect initialRegion = output.faceRegion;
			if (initialRegion.area() > 0) {
				cv::rectangle(img, initialRegion, white, 2);

				int offset = 100 - output.positioningProgress;
				cv::rectangle(img, cv::Rect(initialRegion.tl().x - offset, initialRegion.tl().y - offset, initialRegion.width + 2 * offset, initialRegion.height + 2 * offset), white);
			}
		}
		if (output.state == STATE_CAPTURING) {
			cv::Rect initialRegion = output.faceRegion;
			cv::rectangle(img, initialRegion, white, 2);

			cv::rectangle(img, cv::Rect(initialRegion.x, initialRegion.y + initialRegion.height + 5, initialRegion.width, 10), white);
			int w = initialRegion.width * output.capturingProgress / 100;
			cv::rectangle(img, cv::Rect(initialRegion.x, initialRegion.y + initialRegion.height + 5, w, 10), white, -1);
		}

		cv::putText(img, getStateName(output.state), cv::Point(10, 15), CV_FONT_HERSHEY_SIMPLEX, 0.5, white, 1, CV_AA);

		cv::imshow("scan", img);
		cv::waitKey(1);
	}
	cv::destroyAllWindows();
}

std::string ISensor::getStateName(State state) {
	switch (state)
	{
	case STATE_OFF:
		return "off";
	case STATE_IDLE:
		return "idle";
	case STATE_POSITIONING:
		return "positioning";
	case STATE_CAPTURING:
		return "capturing";
	case STATE_CAPTURING_DONE:
		return "capturing done";
	default:
		return "unknown state";
	}
}

std::string ISensor::getPosOutputName(PositioningOutput out) {
	switch (out)
	{
	case POS_NONE:
		return "No face";
	case POS_MOVE_CLOSER:
		return "Move Closer";
	case POS_MOVE_FAR:
		return "Move far";
	case POS_DONTMOVE:
		return ""; // Don't move";
	case POS_LOOK_LEFT:
		return "Look left";
	case POS_LOOK_RIGHT:
		return "Look right";
	case POS_LOOK_UP:
		return "Look up";
	case POS_LOOK_DOWN:
		return "Look down";
	case POS_ROTATE_LEFT:
		return "Rotate left";
	case POS_ROTATE_RIGHT:
		return "Rotate right";
	default:
		return "No face";
	}
}
