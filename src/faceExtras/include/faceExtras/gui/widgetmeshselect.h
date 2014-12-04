#ifndef WIDGETMESHSELECT_H
#define WIDGETMESHSELECT_H

#include <QWidget>
#include <QString>
#include <QListWidgetItem>

namespace Ui {
class WidgetMeshSelect;
}

namespace Face {
namespace GUI {

class WidgetMeshSelect : public QWidget
{
    Q_OBJECT
    
public:
    explicit WidgetMeshSelect(QWidget *parent = 0);
    ~WidgetMeshSelect();
    
public slots:
    void setPath(QString &path, QStringList &filters);

private slots:
    void on_listWidget_itemDoubleClicked(QListWidgetItem *item);

private:
    Ui::WidgetMeshSelect *ui;
};

}
}

#endif // WIDGETMESHSELECT_H
