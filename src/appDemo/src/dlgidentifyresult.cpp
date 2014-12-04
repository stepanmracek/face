#include "dlgidentifyresult.h"
#include "ui_dlgidentifyresult.h"

#include <QDebug>

DlgIdentifyResult::DlgIdentifyResult(const QMap<int, Face::Biometrics::MultiExtractor::ComparisonResult> &result,
                                     const FrmKinectMain::Database &database, double threshold, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DlgIdentifyResult)
{
    ui->setupUi(this);

    int selectedID = -1;
    double bestScore = 1e300;

    foreach(int id, result.keys())
    {
        double rawDistance = result[id].distance;
        double convertedDistance = (-rawDistance * 1000) + 10000;

        ui->pteResult->appendPlainText(database.mapIdToName.at(id) + ": " + QString::number(rawDistance) + " " + QString::number(convertedDistance));

        if (rawDistance < bestScore && rawDistance < threshold)
        {
            bestScore = rawDistance;
            selectedID = id;
        }

        foreach (const Face::Biometrics::ScoreLevelFusionBase::Result &r, result[id].perReferenceResults)
        {
            QString scores("  (");
            foreach (double s, r.normalized)
            {
                scores += QString::number(s) + ", ";
            }
            scores += ") => " + QString::number(r.score);

            ui->pteResult->appendPlainText(scores);
        }
    }

    if (selectedID > 0)
    {
        ui->lblResult->setText(database.mapIdToName.at(selectedID));
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
