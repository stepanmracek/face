#ifndef DLGENROLL_H
#define DLGENROLL_H

#include <QDialog>
#include <QMap>

#include "facelib/mesh.h"
#include "biometrics/facetemplate.h"
#include "kinectsensorplugin.h"

namespace Ui {
class DlgEnroll;
}

class DlgEnroll : public QDialog
{
    Q_OBJECT

public:
    explicit DlgEnroll(QMap<int, QString> &mapIdToName, QMap<QString, int> mapNameToId,
                       QHash<int, Face3DTemplate*> database, const FaceClassifier &classifier,
                       KinectSensorPlugin &sensor,
                       QWidget *parent);

    ~DlgEnroll();

private slots:
    void on_btnAdd_clicked();
    void on_btnRemove_clicked();
    void on_listScans_itemSelectionChanged();
    void on_buttonBox_accepted();

    void on_btnExport_clicked();

private:
    KinectSensorPlugin &sensor;
    Ui::DlgEnroll *ui;
    QMap<int, QString> &mapIdToName;
    QMap<QString, int> &mapNameToId;
    QHash<int, Face3DTemplate*> &database;
    const FaceClassifier &classifier;

    QList<Mesh*> scans;
};

#endif // DLGENROLL_H
