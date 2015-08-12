#include "winscans.h"
#include "ui_winscans.h"

#include <QFileInfo>
#include <QInputDialog>
#include <QFileDialog>
#include <QDateTime>
#include <QPlainTextEdit>
#include <QProgressDialog>
#include <QListWidgetItem>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceExtras/gui/dialogscan.h"

using namespace Face;
using namespace Face::Sensors;

WinScans::WinScans(Face::Sensors::ISensor::Ptr sensor, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::WinScans),
    sensor(sensor),
	icpAligner(new Face::FaceData::FaceAlignerIcp()),
	landmarkAligner(new Face::FaceData::FaceAlignerLandmark()),
    lastId(1)
{
    ui->setupUi(this);
	icpAligner->setEnableThreadPool(true);
}

WinScans::~WinScans()
{
    delete ui;
}

void WinScans::addScan(const QString &scanName, const Scan &scan)
{
    if (scan.mesh.pointsMat.rows == 0) return;
	scans[scanName] = scan;
    ui->listWidget->addItem(scanName);
}

void WinScans::on_btnAdd_clicked()
{
	//Face::GUI::DialogScan dlg(sensor);
	//dlg.exec();
	//return;

    ui->btnAdd->setEnabled(false);
    sensor->scan();
	const auto &data = sensor->sensorData();

    if (data.rawScan.mesh.pointsMat.rows == 0 && data.processedScan.mesh.pointsMat.rows == 0) {
		ui->btnAdd->setEnabled(true);
		return;
	}

    bool ok;
    int id = QInputDialog::getInt(this, "ID of subject", "ID", lastId, 1, 10000, 1, &ok);
    if (ok)
	{
		lastId = id;
		QDateTime now = QDateTime::currentDateTime();
		QString scanNumber = now.toString("yyyyMMddHHmmss");
		QString scanName = QString::number(id) + "-" + scanNumber;

		addScan(scanName, data.rawScan);
		addScan(scanName + "avg", data.processedScan);
	}

    ui->btnAdd->setEnabled(true);
}

void WinScans::on_listWidget_itemSelectionChanged()
{
    ui->widget->clearAll();
    if (ui->listWidget->selectedItems().count() > 0)
    {
        QString selection = ui->listWidget->selectedItems()[0]->text();
		ui->widget->addLandmarks(&(scans[selection].landmarks));
		ui->widget->addFace(&(scans[selection].mesh));
    }
    else
    {
        ui->widget->updateGL();
    }
}

void WinScans::on_btnRemove_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    for(QListWidgetItem *i : items)
    {
		scans.remove(i->text());
    }

    qDeleteAll(ui->listWidget->selectedItems());
}

void WinScans::on_btnSaveAll_clicked()
{
    /*QStringList types; types << "obj" << "binz";
    bool ok;
    QString selection = QInputDialog::getItem(this, "Select type", "Select type", types, 0, false, &ok);
    if (!ok) return;*/

    QString directory = QFileDialog::getExistingDirectory(this);
    if (directory.isEmpty() || directory.isNull()) return;
    if (!directory.endsWith(QDir::separator())) directory.append(QDir::separator());

	auto keys = scans.keys();
	int n = keys.size();

	QProgressDialog progressDlg("Saving scans", "Cancel", 0, n, this);
	progressDlg.setWindowModality(Qt::WindowModal);

	for (int i = 0; i < n; i++)
    {
		progressDlg.setValue(i);
		const QString &scanName = keys[i];
		const auto &scan = scans[scanName];
		scan.serialize((directory + scanName).toStdString(), false);

        /*if (selection.compare("binz") == 0)
        {
            scan.mesh.writeBINZ((directory + scanName + ".binz").toStdString());
        }
        else
        {
            scan.mesh.writeOBJ((directory + scanName + ".obj").toStdString());
        }

		if (!scan.landmarks.points.empty())
			scan.landmarks.serialize((directory + scanName).toStdString() + ".yml");*/

		if (progressDlg.wasCanceled())
			break;
    }
	progressDlg.setValue(n);
}

void Face::Sensors::WinScans::on_btnMDenoise_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    for(QListWidgetItem *i : items)
    {
        Face::FaceData::SurfaceProcessor::mdenoising(scans[i->text()].mesh, 0.01f, 10, 10);
        ui->widget->updateGL();
    }
}

void Face::Sensors::WinScans::on_btnZSmooth_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    for(QListWidgetItem *i : items)
    {
        Face::FaceData::SurfaceProcessor::zsmooth(scans[i->text()].mesh, 0.2, 5);
        ui->widget->updateGL();
    }
}

void WinScans::on_btnLoad_clicked()
{
    QString directory = QFileDialog::getExistingDirectory(this);
    if (directory.isEmpty() || directory.isNull()) return;

    std::vector<std::string> filters;
    filters.push_back("*.binz");
    filters.push_back("*.bin");
    filters.push_back("*.obj");
    std::vector<std::string> files = Face::LinAlg::Loader::listFiles(directory.toStdString(), filters, Face::LinAlg::Loader::AbsoluteFull);
	int n = files.size();

	QProgressDialog progressDlg("Loading scans", "Cancel", 0, n, this);
	progressDlg.setWindowModality(Qt::WindowModal);

	for (int i = 0; i < n; i++)
    {
		progressDlg.setValue(i);
		const std::string &path = files[i];
        QFileInfo info(QString::fromStdString(path));
		Scan s(path);
		addScan(info.baseName(), s);

		if (progressDlg.wasCanceled())
			break;
    }
	progressDlg.setValue(n);
}


void Face::Sensors::WinScans::on_btnRemoveTexture_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    for(QListWidgetItem *i : items)
    {
        scans[i->text()].mesh.colors.clear();
        ui->widget->updateGL();
    }
}

void Face::Sensors::WinScans::on_btnIcpAlign_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    for(QListWidgetItem *i : items)
    {
        icpAligner->align(scans[i->text()].mesh, 100, Face::FaceData::FaceAlignerIcp::TemplateMatching);
        ui->widget->updateGL();
    }
}

void Face::Sensors::WinScans::on_btnLmAlign_clicked()
{
	QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
	for(QListWidgetItem *i : items)
	{
		landmarkAligner->align(scans[i->text()].mesh, scans[i->text()].landmarks);
		ui->widget->updateGL();
	}
}

/*void Face::Sensors::WinScans::on_btnCrop_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    foreach(QListWidgetItem *i, items)
    {
        scans[i->text()].mesh = scans[i->text()].mesh.radiusSelect(80);
        ui->widget->updateGL();
    }
}*/

namespace
{
	std::ostream & operator<< (std::ostream &out, cv::Rect const &r)
	{
		if (r.area() > 0)
			out << std::to_string(r.x) << ";" << std::to_string(r.y) << ": " << std::to_string(r.width) << "x" << std::to_string(r.height);
		else
			out << "-";

		return out;
	}
}

void Face::Sensors::WinScans::on_btnInfo_clicked()
{
	QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
	if (items.count() < 1) return;
	const auto &scan = scans[items.at(0)->text()];

	std::stringstream ss;
	ss << "Points: " << scan.mesh.pointsMat.rows << std::endl;
	ss << "Triangles: " << scan.mesh.triangles.size() << std::endl;
	ss << "Colors: " << (scan.mesh.colors.size() > 0 ? "Yes" : "No") << std::endl;
	ss << "UV map: " << (scan.mesh.uvmap.size() > 0 ? "Yes" : "No") << std::endl;
	ss << "Landmarks: " << scan.landmarks.points.size() << std::endl;
	ss << "External texture: " << (scan.texture.rows > 0 && scan.texture.cols > 0 ? std::to_string(scan.texture.cols) + "x" + std::to_string(scan.texture.rows) : "No") << std::endl;
	ss << "Face region: " << scan.faceRegion << std::endl;
 
	QDialog dlg;
	QPlainTextEdit *infoBox = new QPlainTextEdit(&dlg);
	infoBox->setPlainText(QString::fromStdString(ss.str()));
	dlg.setLayout(new QBoxLayout(QBoxLayout::TopToBottom, &dlg));
	dlg.layout()->addWidget(infoBox);
	dlg.exec();
}

void Face::Sensors::WinScans::on_btnRename_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    if (items.count() < 1) return;

    bool ok;
    QString oldText = items.at(0)->text();
    QString newText = QInputDialog::getText(this, "New name", "New name", QLineEdit::Normal, oldText, &ok);

    if (ok && !newText.isEmpty() && !newText.isNull())
    {
        Scan scan = scans[oldText];
        scans.remove(oldText);

        qDeleteAll(ui->listWidget->selectedItems());
        addScan(newText, scan);

        //items[0]->setText(newText);
        //scans[newText] = scan;
        //ui->listWidget->setCurrentItem(items.at(0), QItemSelectionModel::ClearAndSelect);
        //ui->listWidget->setFocus();
    }
}
