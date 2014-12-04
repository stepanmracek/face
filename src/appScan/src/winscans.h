#ifndef WINSCANS_H
#define WINSCANS_H

#include <QMainWindow>
#include <QMap>

#include "faceSensors/isensor.h"
#include "faceCommon/facedata/facealigner.h"
#include "faceCommon/facedata/landmarks.h"

namespace Ui {
class WinScans;
}

namespace Face {
namespace Sensors {

class WinScans : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit WinScans(Face::Sensors::ISensor::Ptr sensor, Face::FaceData::FaceAligner::Ptr aligner, QWidget *parent = 0);
    ~WinScans();
    
private slots:
    void on_btnAdd_clicked();

    void on_listWidget_itemSelectionChanged();

    void on_btnRemove_clicked();

    void on_btnSaveAll_clicked();

    void on_btnLoad_clicked();

    void on_btnZSmooth_clicked();

    void on_btnMDenoise_clicked();

    void on_btnRemoveTexture_clicked();

    void on_btnAlign_clicked();

    void on_btnCrop_clicked();

private:
    void addFace(const QString &scanName, const Face::FaceData::Mesh &mesh);

    Ui::WinScans *ui;
    int lastId;
    Face::Sensors::ISensor::Ptr sensor;
    Face::FaceData::FaceAligner::Ptr aligner;
    QMap<QString, Face::FaceData::Mesh> meshes;
    QMap<QString, Face::FaceData::Landmarks> landmarks;
};

}
}

#endif // WINSCANS_H
