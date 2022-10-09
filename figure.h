#ifndef FIGURE_H
#define FIGURE_H

#include <vector>

struct Point3D {
    int x, y, z;
};


struct SDL_Renderer;

struct Edge
{
    Point3D p1;
    Point3D p2;
};

class Figure
{
public:
    Figure();
    Figure(const std::vector<Point3D>& points);

    void SetPoints(const std::vector<Point3D>& points);

    void DrawOutline(SDL_Renderer* renderer);

    int MaxY() const;

    const std::vector<Point3D> &Points() const;

    size_t ScanLines() const;

    const std::vector<Edge> &Edges() const;

    int getA() const;

    int getB() const;

    int getC() const;

    int getD() const;

private:

    std::vector<Point3D> m_Points;

    size_t              m_nScanLines;
    std::vector<Edge>   m_Edges;
    int m_MaxY;
    int a;
    int b;
    int c;
    int d;

    void Init();
};

#endif // FIGURE_H
