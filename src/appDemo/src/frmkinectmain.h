#ifndef FRMKINECTMAIN_H
#define FRMKINECTMAIN_H

#include <QMainWindow>
#include <QHash>
#include <QMap>

#include "faceCommon/biometrics/multiextractor.h"
#include "faceCommon/facedata/facealigner.h"
#include "faceCommon/facedata/faceprocessor.h"
#include "faceSensors/isensor.h"

namespace Ui {
class FrmKinectMain;
}

class FrmKinectMain : public QMainWindow
{
    Q_OBJECT

public:
    struct Database
    {
        std::map<int, QString> mapIdToName;
        std::map<QString, int> mapNameToId;
        std::map<int, std::vector<Face::Biometrics::MultiTemplate>> scans;
    };

    explicit FrmKinectMain(QWidget *parent = 0);
    ~FrmKinectMain();

private slots:
    void on_btnProperties_clicked();
    void on_btnDelete_clicked();
    void on_btnIdentify_clicked();
    void on_btnVerify_clicked();
    void on_btnEnroll_clicked();
    void on_listDatabase_itemSelectionChanged();

    void on_btnExport_clicked();

    void on_sliderRaw_valueChanged(double value);

    void on_sliderConverted_valueChanged(double value);

    void on_toolButton_clicked();

private:
    Ui::FrmKinectMain *ui;

    Face::Sensors::ISensor::Ptr sensor;
    Face::Biometrics::MultiExtractor::Ptr extractor;
    Face::FaceData::FaceAligner::Ptr aligner;
    Face::FaceData::FaceProcessor::Ptr processor;
    Database database;

    void importDirectory(const QString &dirPath);
    void refreshList();
    void setMainButtonsState();
};

#endif // FRMKINECTMAIN_H
