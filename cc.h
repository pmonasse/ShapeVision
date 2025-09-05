// SPDX-License-Identifier: MPL-2.0
/**
 * @file cc.h
 * @brief Contours & Continua
 * @author Pascal Monasse <pascal.monasse@enpc.fr>
 * @date 2025
 */

#ifndef CC_H
#define CC_H

#include <vector>

template <typename T>
struct Point {
    T x, y;
    Point(): x(-1), y(-1) {}
    Point(T x0, T y0): x(x0), y(y0) {}
    Point(const Point<short int>& p): x(p.x), y(p.y) {}
    Point<T>& operator=(const Point<T>& p) { x=p.x; y=p.y; return *this; }
    bool operator==(const Point& p) const { return x==p.x && y==p.y; }
    bool operator!=(const Point& p) const { return !(*this == p); }
};

typedef Point<double> DPoint;
typedef Point<short int> Pos;

struct Contour {
    Pos parent; ///< Identify merges
    DPoint p;
    float lvl;
};

struct Continuum {
    int parent; ///< Identify merges
    int infCtr, supCtr; ///< Inf and sup contour indexes
    std::vector<DPoint> mme; ///< Monotone mesh elements
    Continuum(int inf, int sup): parent(-1), infCtr(inf), supCtr(sup) {}
};

/// Contours and continua
struct CC {
    Contour* contours;
    std::vector<Continuum> continua;
    int w,h;
    CC(const float* im, int w, int h);

    int idx(int x, int y) const { return y*w+x; }
    int idx(Pos p) const { return idx(p.x,p.y); }

    Pos create_saddle(Pos p, float lvl[4]);
    int create_continuum(Pos inf, Pos sup, const DPoint& p);

    void merge_contours(Pos c1, Pos c2);
    Pos root_contour(Pos c);
};

#endif
