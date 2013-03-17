#include "linalg/delaunay.h"

#include <QDebug>
#include <QMap>
#include <QPair>
#include <QVector>

#include <opencv2/opencv.hpp>
#include <climits>

QVector<cv::Vec3i> Delaunay::process(QVector<cv::Point2d> &points)
{
    int n = points.count();
    QVector<cv::Vec3i> result;

    int minx = INT_MAX;
    int maxx = INT_MIN;
    int miny = INT_MAX;
    int maxy = INT_MIN;

    for (int i = 0; i < n; i++)
    {
        int x = points[i].x;
        int y = points[i].y;
        if (x > maxx) maxx = x;
        if (x < minx) minx = x;
        if (y > maxy) maxy = y;
        if (y < miny) miny = y;
    }

    minx -= 1;
    miny -= 1;
    maxx += 1;
    maxy += 1;

    qDebug() << "Delaunay in x =" << minx << ".." << maxx << "; y =" << miny << ".." << maxy << "; |points| =" << n;
    QMap<QPair<float, float>, int> coord2Index;

    CvRect rect = cv::Rect(minx, miny, maxx-minx, maxy-miny);
    cv::Subdiv2D subdiv(rect);
    std::vector<cv::Point2f> fPoints(n);
    for (int i = 0; i < n; i++)
    {
        float x = points[i].x;
        float y = points[i].y;
        fPoints[i] = cv::Point2f(x, y);
        coord2Index[QPair<float, float>(x, y)] = i;
    }
    subdiv.insert(fPoints);

    std::vector<cv::Vec6f> triangleList;
    subdiv.getTriangleList(triangleList);

    //int edge;
    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        cv::Vec6f &triangle = triangleList.at(i);
        //int v1,v2,v3;
        cv::Point2f p1(triangle[0],triangle[1]);
        cv::Point2f p2(triangle[2],triangle[3]);
        cv::Point2f p3(triangle[4],triangle[5]);

        if (p1.x >= maxx) continue;
        if (p1.y >= maxy) continue;
        if (p2.x >= maxx) continue;
        if (p2.y >= maxy) continue;
        if (p3.x >= maxx) continue;
        if (p3.y >= maxy) continue;

        if (p1.x <= minx) continue;
        if (p1.y <= miny) continue;
        if (p2.x <= minx) continue;
        if (p2.y <= miny) continue;
        if (p3.x <= minx) continue;
        if (p3.y <= miny) continue;

        //subdiv.locate(p1, edge, v1);
        //subdiv.locate(p2, edge, v2);
        //subdiv.locate(p3, edge, v3);

        cv::Vec3i triangleIndicies;
        triangleIndicies[0] = coord2Index[QPair<float, float>(p1.x, p1.y)];
        triangleIndicies[1] = coord2Index[QPair<float, float>(p2.x, p2.y)];
        triangleIndicies[2] = coord2Index[QPair<float, float>(p3.x, p3.y)];

        result.append(triangleIndicies);
    }
    return result;

    /*qDebug() << "Reading edges";
    QSet<QString> used;
    CvSeqReader reader;
    int total = subdiv->edges->total;
    cvStartReadSeq((CvSeq*)subdiv->edges, &reader);
    for (int i = 0; i < total; i++)
    {
        CvQuadEdge2D *edge = (CvQuadEdge2D *)reader.ptr;
        CvGraphEdge *ge = (CvGraphEdge *)reader.ptr;

        if(CV_IS_SET_ELEM(edge))
        {
            CvSubdiv2DPoint * org = cvSubdiv2DEdgeOrg( (CvSubdiv2DEdge) edge );
            CvSubdiv2DPoint * dst = cvSubdiv2DEdgeDst( (CvSubdiv2DEdge) edge );
            if(org && dst && CV_SUBDIV2D_POINT_IS_DELAUNAY(org))
            {
                CvSubdiv2DEdge e =  (CvSubdiv2DEdge)edge;

                addTris(cvSubdiv2DGetEdge( e, CV_NEXT_AROUND_LEFT), result, used);
                addTris(cvSubdiv2DGetEdge( e, CV_PREV_AROUND_ORG), result, used);
            }
        }
        CV_NEXT_SEQ_ELEM(subdiv->edges->elem_size, reader);
    }

    qDebug() << "Delaunay done; |triangles| =" << result.count();
    cvReleaseMemStorage(&storage);
    return result;*/
}
