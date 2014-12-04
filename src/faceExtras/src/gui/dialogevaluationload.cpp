#include "faceExtras/gui/dialogevaluationload.h"
#include "ui_dialogevaluationload.h"

#include <QFileDialog>

using namespace Face::GUI;

DialogEvaluationLoad::DialogEvaluationLoad(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogEvaluationLoad)
{
    ui->setupUi(this);
}

DialogEvaluationLoad::~DialogEvaluationLoad()
{
    delete ui;
}

void DialogEvaluationLoad::on_pbtnLoadGenuines_clicked()
{
    ui->leGenuineScores->setText(QFileDialog::getOpenFileName(this, "Load genuine scores"));
}

void DialogEvaluationLoad::on_pbtnLoadImpostors_clicked()
{
    ui->leImpostorScores->setText(QFileDialog::getOpenFileName(this, "Load impostor scores"));
}

QPair<QString, Face::Biometrics::Evaluation> DialogEvaluationLoad::getResult()
{
    if (ui->leName->text().isEmpty() ||
        ui->leGenuineScores->text().isEmpty() ||
        ui->leImpostorScores->text().isEmpty())
    {
        return QPair<QString, Face::Biometrics::Evaluation>();
    }

    auto gen = Face::LinAlg::Vector::fromFile(ui->leGenuineScores->text().toStdString()).toStdVector();
    auto imp = Face::LinAlg::Vector::fromFile(ui->leImpostorScores->text().toStdString()).toStdVector();
    return QPair<QString, Face::Biometrics::Evaluation>(ui->leName->text(), Face::Biometrics::Evaluation(gen, imp));
}
