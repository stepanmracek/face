#ifndef DLGIDENTIFYRESULT_H
#define DLGIDENTIFYRESULT_H

#include <QDialog>

#include <frmkinectmain.h>

namespace Ui {
class DlgIdentifyResult;
}

class DlgIdentifyResult : public QDialog
{
    Q_OBJECT

public:
    explicit DlgIdentifyResult(const QMap<int, Face::Biometrics::MultiExtractor::ComparisonResult> &result,
                               const FrmKinectMain::Database &database, double threshold, QWidget *parent = 0);
    ~DlgIdentifyResult();

private:
    Ui::DlgIdentifyResult *ui;
};

#endif // DLGIDENTIFYRESULT_H
