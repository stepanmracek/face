#ifndef DLGREFERENCEPROPERTIES_H
#define DLGREFERENCEPROPERTIES_H

#include <QDialog>
#include <QMap>

#include "faceCommon/biometrics/multitemplate.h"
#include "frmkinectmain.h"

namespace Ui {
class DlgReferenceProperties;
}

class DlgReferenceProperties : public QDialog
{
    Q_OBJECT

public:
    explicit DlgReferenceProperties(const QString &name, const FrmKinectMain::Database &database, QWidget *parent = 0);
    ~DlgReferenceProperties();

private:
    Ui::DlgReferenceProperties *ui;
};

#endif // DLGREFERENCEPROPERTIES_H
