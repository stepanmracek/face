#include "dlgidentifyresult.h"
#include "ui_dlgidentifyresult.h"

#include <QDebug>

DlgIdentifyResult::DlgIdentifyResult(const QMap<int, double> &result, QMap<int, QString> &mapIdToName, double threshold, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DlgIdentifyResult)
{
    ui->setupUi(this);

    int selectedID = -1;
    double bestScore = 1e300;

    foreach(int id, result.keys())
    {
        ui->listResults->addItem(mapIdToName[id] + ": " + QString::number(result[id]));

        if (result[id] < bestScore && result[id] < threshold)
        {
            bestScore = result[id];
            selectedID = id;
        }
    }

    if (selectedID > 0)
    {
        ui->lblResult->setText(mapIdToName[selectedID]);
    }
    else
    {
        ui->lblResult->setText("Not recognized");
    }
}

DlgIdentifyResult::~DlgIdentifyResult()
{
    delete ui;
}
