#ifndef ISENSOR_H
#define ISENSOR_H

#if (defined WIN32 || defined _WIN32 || defined WINCE)
#if (defined FACESENSORS_COMPILING)
#define FACESENSORS_EXPORTS __declspec(dllexport)
#else
#define FACESENSORS_EXPORTS __declspec(dllimport)
#endif
#else
#define FACESENSORS_EXPORTS
#endif

#include <Poco/ClassLoader.h>

#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/landmarks.h"

namespace Face {
namespace Sensors {

struct FACESENSORS_EXPORTS Scan
{
	Face::FaceData::Mesh mesh;
	Face::FaceData::Landmarks landmarks;
	//Face::FaceData::Landmarks allPoints;
	ImageBGR texture;
	cv::Rect faceRegion;

	Scan() {}
	Scan(const std::string &path);
	
	void serialize(const std::string &path, bool obj) const;
};

struct FACESENSORS_EXPORTS SensorData
{
	Scan rawScan;
	Scan processedScan;
	ImageGrayscale depthImage;
};

class FACESENSORS_EXPORTS ISensor
{
public:
    typedef cv::Ptr<ISensor> Ptr;

	enum State { STATE_OFF, STATE_IDLE, STATE_POSITIONING, STATE_CAPTURING, STATE_CAPTURING_DONE };

	enum PositioningOutput {
		POS_NONE, POS_MOVE_CLOSER, POS_MOVE_FAR,
		POS_LOOK_LEFT, POS_LOOK_RIGHT,
		POS_LOOK_UP, POS_LOOK_DOWN,
		POS_ROTATE_LEFT, POS_ROTATE_RIGHT,
		POS_DONTMOVE
	};

	struct Output
	{
		State state;
		State previousState;
		PositioningOutput positioningOutput;
		ImageGrayscale currentFrame;
		int capturingProgress;
		int positioningProgress;
		cv::Rect faceRegion;
		bool stopGesture;

		Output()
		{
			currentFrame = ImageGrayscale::zeros(480, 640);
		}
	};

protected:
	Output output;

public:
	const Output &getOutput() { return output; }
	virtual void doLoop() = 0;
	virtual void start() = 0;
	virtual void stop() = 0;
	virtual SensorData sensorData() = 0;
	void scan();

	virtual ~ISensor() {}

	std::string getStateName(State state);
	std::string getPosOutputName(PositioningOutput out);

};

class FACESENSORS_EXPORTS SensorLoader
{
private:
    typedef Poco::ClassLoader<Face::Sensors::ISensor> SensorClassLoader;
    typedef Poco::Manifest<Face::Sensors::ISensor> SensorManifest;
    std::vector<std::string> sensorNames;
    SensorClassLoader loader;

public:
    SensorLoader(const std::string pluginDirectory = ".");

    std::vector<std::string> getSensorNames() {return sensorNames; }
    ISensor::Ptr getSensor(const std::string &sensorName);
};
}
}

#endif // ISENSOR_H
