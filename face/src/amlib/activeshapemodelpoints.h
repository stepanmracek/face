#ifndef ACTIVESHAPEMODELPOINTS_H
#define ACTIVESHAPEMODELPOINTS_H

#include <QList>
#include <QFile>
#include <QTextStream>

#include <cassert>

enum ActiveShapeModelPointType
{
    pointType_begin,
    pointType_end,
    pointType_line,
    pointType_tjunction
};

class ActiveShapeModelPoint
{
public:
    ActiveShapeModelPointType type;
    int point1;
    int point2;
    int point3;
};

class ActiveShapeModelPointDefinition
{
public:
    QList<ActiveShapeModelPoint> points;

    void fromFile(const char *path)
    {
        points.clear();
        QFile f(path);
        bool exists = f.exists();
        assert(exists);
        bool opened = f.open(QIODevice::ReadOnly);
        assert(opened);
        QTextStream in(&f);

        while (!in.atEnd())
        {
            QString type;
            in >> type;

            if (type == "") break;

            //qDebug() << type;

            char charType = type.at(0).toAscii();
            bool valid = (charType == 'b') || (charType == 'e') || (charType == 'l') || (charType == 't');
            assert(valid);

            ActiveShapeModelPoint point;

            switch (charType)
            {
            case 'b':
                point.type = pointType_begin;
                in >> (point.point1);
                //qDebug() << "begin" << point.point1;
                break;
            case 'e':
                point.type = pointType_end;
                in >> (point.point1);
                //qDebug() << "end" << point.point1;
                break;
            case 'l':
                point.type = pointType_line;
                in >> (point.point1);
                in >> (point.point2);
                //qDebug() << "line" << point.point1 << point.point2;
                break;
            case 't':
                point.type = pointType_tjunction;
                in >> (point.point1);
                in >> (point.point2);
                in >> (point.point3);
                break;
            }

            points.append(point);
        }
    }
};

#endif // ACTIVESHAPEMODELPOINTS_H
