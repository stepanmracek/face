#include "dlglaunchproperties.h"
#include "ui_dlglaunchproperties.h"

#include <QFileDialog>
#include <QDebug>

#include "faceCommon/settings/settings.h"

DlgLaunchProperties::DlgLaunchProperties(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DlgLaunchProperties)
{
    ui->setupUi(this);

	auto sensorNames = loader.getSensorNames();
	for (const std::string &sn : sensorNames)
		ui->cmbSensor->addItem(QString::fromStdString(sn));

	auto &s = Face::Settings::instance();
	ui->leHaarPath->setText(QString::fromStdString(s.settingsMap[Face::Settings::CascadeFaceDetectorPathKey]));
	ui->leMeanFacePath->setText(QString::fromStdString(s.settingsMap[Face::Settings::MeanFaceModelPathKey]));
	ui->lePreAlignTemplatePath->setText(QString::fromStdString(s.settingsMap[Face::Settings::PreAlignTemplatePathKey]));
	ui->leLandmarkPath->setText(QString::fromStdString(s.settingsMap[Face::Settings::MeanFaceModelLandmarksPathKey]));
	ui->leMultiExtractorPath->setText(QString::fromStdString(s.settingsMap[Face::Settings::MultiExtractorPathKey]));
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

void DlgLaunchProperties::on_pbLandmarksPath_clicked()
{
	QString lmPath = QFileDialog::getOpenFileName(this, "Open landmarks file");
	if (!lmPath.isNull())
	{
		ui->leLandmarkPath->setText(lmPath);
	}
}

void DlgLaunchProperties::on_buttonBox_accepted()
{
	auto &s = Face::Settings::instance();
	s.settingsMap[Face::Settings::CascadeFaceDetectorPathKey] = ui->leHaarPath->text().toStdString();
	s.settingsMap[Face::Settings::MeanFaceModelPathKey] = ui->leMeanFacePath->text().toStdString();
	s.settingsMap[Face::Settings::PreAlignTemplatePathKey] = ui->lePreAlignTemplatePath->text().toStdString();
	s.settingsMap[Face::Settings::MeanFaceModelLandmarksPathKey] = ui->leLandmarkPath->text().toStdString();
	s.settingsMap[Face::Settings::MultiExtractorPathKey] = ui->leMultiExtractorPath->text().toStdString();
}

Face::Sensors::ISensor::Ptr DlgLaunchProperties::getSensor()
{
	return loader.getSensor(ui->cmbSensor->currentText().toStdString());
    return 0;
}
