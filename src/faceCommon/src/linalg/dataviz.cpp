#include "faceCommon/linalg/dataviz.h"

#include <fstream>

using namespace Face::LinAlg;

void DataViz::ToGnuplotFile(std::vector<Matrix> &vectors, const std::string &path)
{
    int n = vectors.size();
    if (n == 0) return;
    int m = vectors[0].rows;

    std::ofstream out(path);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            out << vectors[i](j) << " ";
        }
        out << std::endl;
    }

    out.flush();
    out.close();
}
