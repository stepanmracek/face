#ifndef FRMKINECTMAIN_H
#define FRMKINECTMAIN_H

#include <QMainWindow>
#include <QHash>

#include "biometrics/facetemplate.h"

namespace Ui {
class FrmKinectMain;
}

class FrmKinectMain : public QMainWindow
{
    Q_OBJECT

public:
    explicit FrmKinectMain(const QString &databasePath, const FaceClassifier &classifier, QWidget *parent = 0);
    ~FrmKinectMain();

private slots:
    void on_btnProperties_clicked();

    void on_btnDelete_clicked();

private:
    const FaceClassifier &classifier;
    Ui::FrmKinectMain *ui;
    QHash<int, QString> hashIdToName;
    QHash<QString, int> hashNameToId;
    QHash<int, FaceTemplate> database;

    void initDatabase(const QString &dirPath);

};

#endif // FRMKINECTMAIN_H
