#ifndef DLGENROLL_H
#define DLGENROLL_H

#include <QDialog>
#include <QMap>

#include "faceSensors/isensor.h"
#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/faceprocessor.h"
#include "faceCommon/biometrics/multiextractor.h"
#include "frmkinectmain.h"

namespace Ui {
class DlgEnroll;
}

class DlgEnroll : public QDialog
{
    Q_OBJECT

public:
    explicit DlgEnroll(Face::Sensors::ISensor::Ptr sensor,
                       Face::FaceData::FaceProcessor::Ptr processor,
                       Face::Biometrics::MultiExtractor::Ptr &extractor,
                       FrmKinectMain::Database &database,
                       QWidget *parent);

    ~DlgEnroll();

private slots:
    void on_btnAdd_clicked();
    void on_btnRemove_clicked();
    void on_listScans_itemSelectionChanged();
    void on_buttonBox_accepted();

    void on_btnExport_clicked();

private:
    Face::Sensors::ISensor::Ptr sensor;
    Face::FaceData::FaceProcessor::Ptr processor;
    Face::Biometrics::MultiExtractor::Ptr extractor;
    FrmKinectMain::Database &database;

    Ui::DlgEnroll *ui;

    QList<Face::FaceData::Mesh*> scans;
};

#endif // DLGENROLL_H
