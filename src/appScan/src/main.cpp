#include <QApplication>
#include <QInputDialog>


#include "kinect/kinectsensorplugin.h"
#include "softkinetic/ds325sensorfactory.h"
#include "softkinetic/ds325sensor.h"
#include "occipital/occipitalsensor.h"

#include "faceExtras/gui/glwidget.h"
#include "winscans.h"
#include "faceSensors/isensor.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Face::Sensors::ISensor::Ptr sensor;

    bool ok;
    QStringList sensorNames; sensorNames << "occipital" << "softKinetic" << "kinect";
    QString selection = QInputDialog::getItem(0, "Select sensor", "Sensor:", sensorNames, 0, false, &ok);
    if (!ok) return 0;

    Face::Sensors::SoftKinetic::DS325SensorFactory ds325factory;

    if (selection == "softKinetic")
    {
        sensor = ds325factory.create("haar-face.xml");
    }
    else if (selection == "kinect")
    {
        sensor = new Face::Sensors::Kinect::KinectSensorPlugin("haar-face.xml");
    }
    else if (selection == "occipital")
    {
        sensor = new Face::Sensors::Occipital::OccipitalSensor();
    }

    Face::FaceData::FaceAligner::Ptr aligner =
            new Face::FaceData::FaceAligner(Face::FaceData::Mesh::fromFile("meanForAlign.obj"), "preAlignTemplate.yml");
    Face::Sensors::WinScans win(sensor, aligner);
    win.show();
    return app.exec();
}

