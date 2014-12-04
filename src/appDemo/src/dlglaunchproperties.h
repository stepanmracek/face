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

    QString getHaarPath();
    QString getMultiExtractorPath();
    QString getAlignerPath();
    QString getPreAlignTemplatePath();
    Face::Sensors::ISensor::Ptr getSensor();

private slots:
    void on_pbHaarPath_clicked();

    void on_pbMeanFacePath_clicked();

    void on_pbPreAlignTemplatePath_clicked();

    void on_pbMultiExtractorPath_clicked();

    void on_buttonBox_accepted();

private:
    Ui::DlgLaunchProperties *ui;

    static constexpr char *kinect = "Kinect";
    static constexpr char *occipital = "Occipital";
    static constexpr char *softKinetic = "SoftKinetic";
};

#endif // DLGLAUNCHPROPERTIES_H
