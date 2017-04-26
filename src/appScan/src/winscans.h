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
    explicit WinScans(Face::Sensors::ISensor::Ptr sensor, QWidget *parent = 0);
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

    void on_btnIcpAlign_clicked();

	void on_btnLmAlign_clicked();

	void on_btnInfo_clicked();

    void on_btnRename_clicked();

private:
	void addScan(const QString &scanName, const Scan &scan);

    Ui::WinScans *ui;
    int lastId;
    Face::Sensors::ISensor::Ptr sensor;
    Face::FaceData::FaceAlignerIcp::Ptr icpAligner;
	Face::FaceData::FaceAlignerLandmark::Ptr landmarkAligner;
    QMap<QString, Scan> scans;
};

}
}

#endif // WINSCANS_H
