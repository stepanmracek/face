#ifndef DIALOGSCAN_H
#define DIALOGSCAN_H

#include <QDialog>
#include <QGraphicsScene>
#include <QGraphicsSvgItem>

#include "faceExtras/faceExtras.h"
#include "faceSensors/isensor.h"

namespace Ui {
class DialogScan;
}

namespace Face {
namespace GUI {

class FACEEXTRAS_EXPORTS DialogScan : public QDialog
{
    Q_OBJECT

public:
	explicit DialogScan(Face::Sensors::ISensor::Ptr sensor, QWidget *parent = 0);
	~DialogScan();

	Face::Sensors::SensorData getScan() { return scan; }

private:
	Ui::DialogScan *ui;
	Face::Sensors::ISensor::Ptr sensor;
	Face::Sensors::SensorData scan;
	QGraphicsScene *graphicsScene;
	QGraphicsSvgItem *headTop;
	QGraphicsSvgItem *headSide;
	QGraphicsSvgItem *arrowLeft;
	QGraphicsSvgItem *arrowRight;
	QGraphicsSvgItem *arrowRotate;
	QGraphicsSvgItem *arrowUp;
	QGraphicsSvgItem *arrowDown;
	QGraphicsPixmapItem *previewItem;
	QGraphicsTextItem *stateItem;
	QGraphicsRectItem *faceRoi;
	QGraphicsRectItem *progressFrame;
	QGraphicsRectItem *progressValue;

	void hideAllArrows();

	void showEvent(QShowEvent * event);
	void timerEvent(QTimerEvent *event);
};

}
}

#endif // DIALOGSCAN_H
