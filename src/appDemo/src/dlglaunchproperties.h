#ifndef DLGLAUNCHPROPERTIES_H
#define DLGLAUNCHPROPERTIES_H

#include <QDialog>
#include "faceSensors/isensor.h"

namespace Ui {
class DlgLaunchProperties;
}

class DlgLaunchProperties : public QDialog
{
    Q_OBJECT

public:
    explicit DlgLaunchProperties(QWidget *parent = 0);
    ~DlgLaunchProperties();

    Face::Sensors::ISensor::Ptr getSensor();

private slots:
    void on_pbHaarPath_clicked();

    void on_pbMeanFacePath_clicked();

    void on_pbPreAlignTemplatePath_clicked();

    void on_pbMultiExtractorPath_clicked();

	void on_pbLandmarksPath_clicked();

    void on_buttonBox_accepted();

private:
	Face::Sensors::SensorLoader loader;
    Ui::DlgLaunchProperties *ui;
};

#endif // DLGLAUNCHPROPERTIES_H
