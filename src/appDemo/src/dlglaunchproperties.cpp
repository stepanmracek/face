#include "dlglaunchproperties.h"
#include "ui_dlglaunchproperties.h"

#include <QFileDialog>
#include <QDebug>

#include "kinect/kinectsensorplugin.h"
#include "occipital/occipitalsensor.h"
#include "softkinetic/ds325sensorfactory.h"

DlgLaunchProperties::DlgLaunchProperties(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DlgLaunchProperties)
{
    ui->setupUi(this);

    ui->cmbSensor->addItem(kinect);
    ui->cmbSensor->addItem(softKinetic);
    ui->cmbSensor->addItem(occipital);

    ui->leHaarPath->setText("haar-face.xml");
    ui->leMeanFacePath->setText("meanForAlign.obj");
    ui->lePreAlignTemplatePath->setText("preAlignTemplate.yml");
    ui->leMultiExtractorPath->setText("classifiers");
}

DlgLaunchProperties::~DlgLaunchProperties()
{
    delete ui;
}

void DlgLaunchProperties::on_pbHaarPath_clicked()
{
    QString haar = QFileDialog::getOpenFileName(this, "Open Haar face detector");
    if (!haar.isNull())
    {
        ui->leHaarPath->setText(haar);
    }
}

void DlgLaunchProperties::on_pbMeanFacePath_clicked()
{
    QString mean = QFileDialog::getOpenFileName(this, "Open mean face mesh");
    if (!mean.isNull())
    {
        ui->leMeanFacePath->setText(mean);
    }
}

void DlgLaunchProperties::on_pbPreAlignTemplatePath_clicked()
{
    QString preAlignTemplate = QFileDialog::getOpenFileName(this, "Open pre-align Template");
    if (!preAlignTemplate.isNull())
    {
        ui->lePreAlignTemplatePath->setText(preAlignTemplate);
    }
}

void DlgLaunchProperties::on_pbMultiExtractorPath_clicked()
{
    QString multiExtractor = QFileDialog::getExistingDirectory(this, "Open multi-extractor");
    if (!multiExtractor.isNull())
    {
        ui->leMultiExtractorPath->setText(multiExtractor);
    }
}

void DlgLaunchProperties::on_buttonBox_accepted()
{

}

QString DlgLaunchProperties::getHaarPath()
{
    return ui->leHaarPath->text();
}

QString DlgLaunchProperties::getMultiExtractorPath()
{
    return ui->leMultiExtractorPath->text();
}

QString DlgLaunchProperties::getAlignerPath()
{
    return ui->leMeanFacePath->text();
}

QString DlgLaunchProperties::getPreAlignTemplatePath()
{
    return ui->lePreAlignTemplatePath->text();
}

Face::Sensors::ISensor::Ptr DlgLaunchProperties::getSensor()
{
    qDebug() << ui->cmbSensor->currentText() << kinect << ui->cmbSensor->currentText().compare(kinect);
    if (ui->cmbSensor->currentText().compare(kinect) == 0)
    {
        return new Face::Sensors::Kinect::KinectSensorPlugin(getHaarPath().toStdString());
    }
    else if (ui->cmbSensor->currentText().compare(occipital) == 0)
    {
        return new Face::Sensors::Occipital::OccipitalSensor();
    }
    else
    {
        Face::Sensors::SoftKinetic::DS325SensorFactory factory;
        return factory.create(getHaarPath().toStdString());
    }
}
