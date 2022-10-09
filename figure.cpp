#include <SDL.h>

#include "figure.h"


Figure::Figure() {}

Figure::Figure(const std::vector<Point3D> &points) :
    m_Points(points)
{
    Init();
}

void Figure::SetPoints(const std::vector<Point3D> &points)
{
    m_Points = points;

    Init();
}

void Figure::DrawOutline(SDL_Renderer *renderer)
{
    for (const auto& edge : m_Edges)
        SDL_RenderDrawLine(renderer, edge.p1.x, edge.p1.y, edge.p2.x, edge.p2.y);
}

int Figure::MaxY() const
{
    return m_MaxY;
}

const std::vector<Point3D> &Figure::Points() const
{
    return m_Points;
}

size_t Figure::ScanLines() const
{
    return m_nScanLines;
}

const std::vector<Edge> &Figure::Edges() const
{
    return m_Edges;
}

int Figure::getA() const
{
    return a;
}

int Figure::getB() const
{
    return b;
}

int Figure::getC() const
{
    return c;
}

int Figure::getD() const
{
    return d;
}



void Figure::Init()
{

    // initialize a list of edges
    for (size_t i = 0; i < m_Points.size(); ++i) {
        Point3D prev;
        Point3D cur = m_Points[i];

        if (i != 0) {
            prev = m_Points[i - 1];
        } else {
            prev = m_Points.back();
        }

        m_Edges.push_back({prev, cur});
    }

    // initialize the number of scan lines

    int minY = INT32_MAX;
    int maxY = INT32_MIN;

    for (auto point : m_Points) {
        minY = (point.y < minY)? point.y : minY;
        maxY = (point.y > maxY)? point.y : maxY;
    }

    m_MaxY = maxY;
    m_nScanLines = maxY - minY + 1;

    // initialize coefficients of the equation of a plane
    a = b = c = d = 0;
    for (size_t i = 0; i < m_Points.size(); ++i) {
        Point3D next, cur = m_Points[i];

        if (i != m_Points.size() - 1) {
            next = m_Points[i + 1];
        } else {
            next = m_Points.front();
        }

        a+= (next.y - cur.y)*(cur.z + next.z);
        b+= (next.z - cur.z)*(cur.x + next.x);
        c+= (next.x - cur.x)*(cur.y + next.y);
    }

    for (const auto& p : m_Points)
        d += a * p.x + b * p.y + c * p.z;

    d /= m_Points.size();


}
