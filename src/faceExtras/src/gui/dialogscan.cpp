#include "faceExtras/gui/dialogscan.h"
#include "ui_dialogscan.h"

#include <QImage>
#include <QPixmap>
#include <qgraphicssvgitem.h>

using namespace Face::GUI;

DialogScan::DialogScan(Face::Sensors::ISensor::Ptr sensor, QWidget *parent) :
	sensor(sensor),
    QDialog(parent),
	ui(new Ui::DialogScan)
{
    ui->setupUi(this);
	graphicsScene = new QGraphicsScene(ui->graphicsView);
	ui->graphicsView->setScene(graphicsScene);

	QImage previewImg(640, 480, QImage::Format_Indexed8);
	QPixmap previewPixmap = QPixmap::fromImage(previewImg);
	previewItem = graphicsScene->addPixmap(previewPixmap);

	stateItem = graphicsScene->addText("", QFont("", 20));
	stateItem->setDefaultTextColor(Qt::darkGreen);

	faceRoi = graphicsScene->addRect(0, 0, 0, 0, QPen(Qt::white, 2));
	progressFrame = graphicsScene->addRect(0, 0, 0, 0, QPen(Qt::white, 2));
	progressValue = graphicsScene->addRect(0, 0, 0, 0, QPen(Qt::white), QBrush(Qt::white));
	
	headSide = new QGraphicsSvgItem(":/posImages/head-side.svg");
	headSide->setScale(2.0);
	headSide->setPos(QPointF(260, 240));
	headSide->setVisible(false);
	graphicsScene->addItem(headSide);

	headTop = new QGraphicsSvgItem(":/posImages/head-top.svg");
	headTop->setScale(2.0);
	headTop->setPos(QPointF(260, 240));
	headTop->setVisible(false);
	graphicsScene->addItem(headTop);

	arrowLeft = new QGraphicsSvgItem(":/posImages/left.svg");
	arrowLeft->setScale(2.0);
	arrowLeft->setPos(QPointF(330, 260));
	arrowLeft->setVisible(false);
	graphicsScene->addItem(arrowLeft);

	arrowRight = new QGraphicsSvgItem(":/posImages/right.svg");
	arrowRight->setScale(2.0);
	arrowRight->setPos(QPointF(330, 260));
	arrowRight->setVisible(false);
	graphicsScene->addItem(arrowRight);

	arrowUp = new QGraphicsSvgItem(":/posImages/up.svg");
	arrowUp->setScale(2.0);
	arrowUp->setPos(QPointF(340, 250));
	arrowUp->setVisible(false);
	graphicsScene->addItem(arrowUp);

	arrowDown = new QGraphicsSvgItem(":/posImages/down.svg");
	arrowDown->setScale(2.0);
	arrowDown->setPos(QPointF(340, 250));
	arrowDown->setVisible(false);
	graphicsScene->addItem(arrowDown);

	arrowRotate = new QGraphicsSvgItem(":/posImages/rotate.svg");
	arrowRotate->setScale(2.0);
	arrowRotate->setPos(QPointF(340, 250));
	arrowRotate->setVisible(false);
	graphicsScene->addItem(arrowRotate);
}

DialogScan::~DialogScan()
{
    delete ui;
}

void DialogScan::showEvent(QShowEvent * event)
{
	sensor->start();
	startTimer(50);

	return;

	while (sensor->getOutput().state != sensor->STATE_OFF) {
		const auto &output = sensor->getOutput();
		sensor->doLoop();

		ImageGrayscale img = output.currentFrame.clone();

		if (sensor->getOutput().state == sensor->STATE_IDLE) {
			for (unsigned char *val = img.data; val != img.dataend; val++) {
				*val = *val / 2;
			}
		}

		if (output.stopGesture) {
			//cv::putText(img, "Sensor covered", cv::Point(120, 50), CV_FONT_HERSHEY_SIMPLEX, 1.0, white, 2, CV_AA);
			sensor->stop();
		}

		if (output.state == sensor->STATE_POSITIONING) {
			//cv::putText(img, getPosOutputName(output.positioningOutput), cv::Point(122, 202), CV_FONT_HERSHEY_SIMPLEX, 1.0, black, 3, CV_AA);
			//cv::putText(img, getPosOutputName(output.positioningOutput), cv::Point(120, 200), CV_FONT_HERSHEY_SIMPLEX, 1.0, white, 2, CV_AA);

			cv::Rect initialRegion = output.faceRegion;
			if (initialRegion.area() > 0) {
				//cv::rectangle(img, initialRegion, white, 2);

				int offset = 100 - output.positioningProgress;
				//cv::rectangle(img, cv::Rect(initialRegion.tl().x - offset, initialRegion.tl().y - offset, initialRegion.width + 2 * offset, initialRegion.height + 2 * offset), white);
			}
		}
		if (output.state == sensor->STATE_CAPTURING) {
			//cv::Rect initialRegion = output.faceRegion;
			//cv::rectangle(img, initialRegion, white, 2);

			//cv::rectangle(img, cv::Rect(initialRegion.x, initialRegion.y + initialRegion.height + 5, initialRegion.width, 10), white);
			//int w = initialRegion.width * output.capturingProgress / 100;
			//cv::rectangle(img, cv::Rect(initialRegion.x, initialRegion.y + initialRegion.height + 5, w, 10), white, -1);
		}

		//cv::putText(img, getStateName(output.state), cv::Point(10, 15), CV_FONT_HERSHEY_SIMPLEX, 0.5, white, 1, CV_AA);

		//cv::imshow("scan", img);
		//cv::waitKey(1);
	}
	//cv::destroyAllWindows();
	this->close();
}

void DialogScan::hideAllArrows()
{
	headSide->setVisible(false);
	headTop->setVisible(false);
	arrowLeft->setVisible(false);
	arrowDown->setVisible(false);
	arrowUp->setVisible(false);
	arrowRight->setVisible(false);
	arrowRotate->setVisible(false);
	faceRoi->setVisible(false);
	progressFrame->setVisible(false);
	progressValue->setVisible(false);
}

void DialogScan::timerEvent(QTimerEvent *event)
{
	sensor->doLoop();
	const auto &output = sensor->getOutput();

	QPixmap pixmap = QPixmap::fromImage(QImage(output.currentFrame.data, output.currentFrame.cols, output.currentFrame.rows, QImage::Format_Indexed8));
	previewItem->setPixmap(pixmap);
	
	stateItem->setPlainText(QString::fromStdString(sensor->getStateName(output.state)));

	hideAllArrows();
	if (output.state == sensor->STATE_POSITIONING) {
		if (output.positioningOutput == sensor->POS_MOVE_CLOSER) {
			headSide->setVisible(true);
			arrowRight->setVisible(true);
		}
		else if (output.positioningOutput == sensor->POS_MOVE_FAR) {
			headSide->setVisible(true);
			arrowLeft->setVisible(true);
		}
		else if (output.positioningOutput == sensor->POS_LOOK_LEFT) {
			headTop->setVisible(true);
			arrowUp->setVisible(true);
		}
		else if (output.positioningOutput == sensor->POS_LOOK_RIGHT) {
			headTop->setVisible(true);
			arrowRotate->setVisible(true);
		}
		else if (output.positioningOutput == sensor->POS_LOOK_UP) {
			headSide->setVisible(true);
			arrowUp->setVisible(true);
		}
		else if (output.positioningOutput == sensor->POS_LOOK_DOWN) {
			headSide->setVisible(true);
			arrowDown->setVisible(true);
		}
	}
	else if (output.state == sensor->STATE_CAPTURING) {
		const auto &r = output.faceRegion;
		faceRoi->setVisible(true);
		faceRoi->setRect(QRectF(r.x, r.y, r.width, r.height));

		progressFrame->setVisible(true);
		progressFrame->setRect(r.x, r.y + r.height + 5, r.width, 10);

		progressValue->setVisible(true);
		int w = r.width * output.capturingProgress / 100;
		progressValue->setRect(r.x, r.y + r.height + 5, w, 10);
	}
	
	/*graphicsScene->addPixmap(pixmap);
	graphicsScene->addText(QString::fromStdString(sensor->getStateName(output.state)), QFont("", 20));
	graphicsScene->addItem(new QGraphicsSvgItem("C:\\data\\face\\positioning\\head-side.svg"));*/
}
