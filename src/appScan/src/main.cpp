#include <QApplication>
#include <QInputDialog>
#include <Poco/ClassLoader.h>
#include <Poco/Manifest.h>
#include <Poco/Glob.h>

#include "faceCommon/objectdetection/detector.h"
#include "faceSensors/isensor.h"
#include "faceExtras/gui/glwidget.h"
#include "winscans.h"
#include "faceCommon/linalg/loader.h"


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Face::Settings::instance().settingsMap[Face::Settings::DebugKey] = Poco::DynamicAny(app.arguments().contains("debug"));

	Face::Sensors::SensorLoader loader;
    std::vector<std::string> sensorNames = loader.getSensorNames();

    if (sensorNames.empty()) {
        std::cout << "No sensor plugin found" << std::endl;
        return 1;
    }

    Face::Sensors::ISensor::Ptr sensor;

    bool ok;
    QStringList choices;
    for (const auto &c : sensorNames) choices << QString::fromStdString(c);
    QString selection = QInputDialog::getItem(0, "Select sensor", "Sensor:", choices, 0, false, &ok);
    if (!ok) return 0;
    sensor = loader.getSensor(selection.toStdString());

    Face::Sensors::WinScans win(sensor);
    win.show();
    return app.exec();
}
