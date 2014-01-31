#ifndef FRMKINECTMAIN_H
#define FRMKINECTMAIN_H

#include <QMainWindow>
#include <QHash>
#include <QMap>

#include "biometrics/facetemplate.h"
#include "kinectsensorplugin.h"

namespace Ui {
class FrmKinectMain;
}

class FrmKinectMain : public QMainWindow
{
    Q_OBJECT

public:
    explicit FrmKinectMain(Face::Sensors::Kinect::KinectSensorPlugin &sensor,
                           const Face::Biometrics::FaceClassifier &classifier,
                           const QString &databasePath,
                           QWidget *parent = 0);
    ~FrmKinectMain();

private slots:
    void on_btnProperties_clicked();
    void on_btnDelete_clicked();
    void on_btnIdentify_clicked();
    void on_btnVerify_clicked();
    void on_btnEnroll_clicked();
    void on_listDatabase_itemSelectionChanged();

    void on_btnExport_clicked();

private:
    Ui::FrmKinectMain *ui;
    Face::Sensors::Kinect::KinectSensorPlugin &sensor;
    const Face::Biometrics::FaceClassifier &classifier;
    QMap<int, QString> mapIdToName;
    QMap<QString, int> mapNameToId;
    QHash<int, Face::Biometrics::FaceTemplate*> database;

    void initDatabase(const QString &dirPath);
    void refreshList();
    void setMainButtonsState();
};

#endif // FRMKINECTMAIN_H
