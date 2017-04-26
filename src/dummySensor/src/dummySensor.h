#include "faceSensors/isensor.h"

namespace Face {
namespace Sensors {

class DummySensor : public Face::Sensors::ISensor
{
private:
    Face::FaceData::Mesh mesh;

public:
    DummySensor();
    DummySensor(const std::string &meshPath);

	virtual void doLoop()
	{
		output.state = STATE_OFF;
		output.previousState = STATE_IDLE;
	};

	virtual void start() { output.state = STATE_IDLE; };
	virtual void stop() { output.state = STATE_OFF; };
    virtual SensorData sensorData();
};

}
}
