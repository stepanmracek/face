#ifndef DLGSCANFACE_H
#define DLGSCANFACE_H

#include <QDialog>
#include <QTimer>
#include <opencv2/opencv.hpp>

#include "linalg/common.h"
#include "facetrack/realtimetrack.h"
#include "facelib/mesh.h"

namespace Ui {
class DlgScanFace;
}

class DlgScanFace : public QDialog
{
    Q_OBJECT

public:
    explicit DlgScanFace(const QString &pathToAlignReference, const QString &faceHaarPath, QWidget *parent = 0);
    ~DlgScanFace();
    Mesh *result;

private:
    Ui::DlgScanFace *ui;
    QTimer *timer;
    RealTimeTrack tracker;
    ImageGrayscale frame;
    u_int8_t rgbBuffer[640*480*3];
    double depthBuffer[640*480];
    bool mask[640*480];
    const QString &pathToAlignReference;

private slots:
    void showFace();
    void scan();
};

#endif // DLGSCANFACE_H
