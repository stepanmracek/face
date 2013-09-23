#ifndef DLGENROLL_H
#define DLGENROLL_H

#include <QDialog>
#include <QMap>

#include "facelib/mesh.h"
#include "biometrics/facetemplate.h"

namespace Ui {
class DlgEnroll;
}

class DlgEnroll : public QDialog
{
    Q_OBJECT

public:
    explicit DlgEnroll(QMap<int, QString> &mapIdToName, QMap<QString, int> mapNameToId,
                       QHash<int, FaceTemplate*> database, const FaceClassifier &classifier,
                       const QString &pathToAlignReference, QWidget *parent);

    ~DlgEnroll();

private slots:
    void on_btnAdd_clicked();
    void on_btnRemove_clicked();
    void on_listScans_itemSelectionChanged();
    void on_buttonBox_accepted();

private:
    Ui::DlgEnroll *ui;
    QMap<int, QString> &mapIdToName;
    QMap<QString, int> &mapNameToId;
    QHash<int, FaceTemplate*> &database;
    const FaceClassifier &classifier;
    const QString &pathToAlignReference;

    QList<Mesh*> scans;
};

#endif // DLGENROLL_H
