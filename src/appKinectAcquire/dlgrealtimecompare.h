#ifndef DLGREALTIMECOMPARE_H
#define DLGREALTIMECOMPARE_H

#include <QDialog>

#include "facetrack/realtimetrack.h"
#include "facedata/mesh.h"
#include "biometrics/facetemplate.h"
#include "facedata/facealigner.h"
#include "biometrics/realtimeclassifier.h"

namespace Ui {
class DlgRealTimeCompare;
}

class DlgRealTimeCompare : public QDialog
{
    Q_OBJECT

public:
    explicit DlgRealTimeCompare(Face::Biometrics::RealTimeClassifier *classifier, const QMap<int, QString> &mapIdToName,
                                const QString &faceHaarPath, QWidget *parent = 0);
    ~DlgRealTimeCompare();

private:
    Ui::DlgRealTimeCompare *ui;
    Face::Biometrics::RealTimeClassifier *classifier;
    const QMap<int, QString> &mapIdToName;
    QTimer *timer;
    Face::Tracking::HaarDetector tracker;
    ImageGrayscale frame;
    u_int8_t rgbBuffer[640*480*3];
    double depthBuffer[640*480];
    bool mask[640*480];

private slots:
    void showFace();
};

#endif // DLGREALTIMECOMPARE_H
