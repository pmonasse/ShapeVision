// SPDX-License-Identifier: MPL-2.0
/**
 * @file cc.h
 * @brief Contours & Continua
 * @author Pascal Monasse <pascal.monasse@enpc.fr>
 * @date 2025-2026
 */

#ifndef CC_H
#define CC_H

#include <vector>
#include <list>

template <typename T>
struct Point {
    T x, y;
    Point(): x(-1), y(-1) {}
    Point(T x0, T y0): x(x0), y(y0) {}
    bool operator==(const Point& p) const { return x==p.x && y==p.y; }
    bool operator!=(const Point& p) const { return !(*this == p); }
    T& operator[](int i)       { return i==0? x: y; }
    T  operator[](int i) const { return i==0? x: y; }
};

typedef Point<double> DPoint;
typedef Point<short int> Pos;

struct Contour {
    int parent; ///< Identify merges
    DPoint p;
    float lvl;
    Contour(): parent(-1), lvl(0) {}
};

struct Continuum {
    int parent; ///< Identify merges
    int infCtr, supCtr; ///< Inf and sup contour indexes
    std::vector<DPoint> mme; ///< Monotone mesh elements
    Continuum(int inf, int sup): parent(-1), infCtr(inf), supCtr(sup) {}
};

struct Rect {
    Pos tl, br; ///< Top-left and bottom-right corners of rectangle
    std::list<std::list<int>> chainCode[4];
    Rect(Pos topLeft, Pos bottomRight) : tl(topLeft), br(bottomRight) {}
};

/// Contours and continua
struct CC {
    Contour* contours;
    std::vector<Continuum> continua;
    Rect R;
    int w,h;
    CC(const float* im, int w, int h);

    int idx(int x, int y) const { return y*w+x; }
    int idx(Pos p) const { return idx(p.x,p.y); }
private:
    bool adjacent_rect(const DPoint& p, Pos sep, int o) const;
    DPoint mme_br(const DPoint& p) const;
    Pos create_saddle(Pos p, const float lvl[4]);
    int create_continuum(Pos inf, Pos sup, const DPoint& p);
    int root_contour(int i);
    int root_contour(Pos c) { return root_contour(idx(c)); }
    int root_continuum(int i);
    void merge_contours(Pos c1, Pos c2) { merge_contours(idx(c1),idx(c2)); }
    void merge_contours(int i1, int i2);

    int fill_simple_chainCode(std::list<int>& L, const float lvl[2],
                              const Pos v[2], const DPoint& dtl);
    Rect build_mme(Pos p, float lvl[4]);
    void insert_chainCode(std::list<int>& L, int iSplit, int iCtn);
    void mark_exit(Rect& R, int iSplit, int iCtn, int iSideIn);
    bool mark_exit_side(std::list<int>& L, float v, int iSplit, int iCtn);
    void split_continuum(Rect& Rsrc, Rect& Rdst,
                         std::vector<DPoint>::iterator it, Pos sep,
                         int iSplit, int iCtn, int iSideIn);
    void propagate(Rect& R1, Rect& R2, Pos sep, int o,
                   const std::list<int>& L1, const std::list<int>& L2);
    Rect merge_rectangles(Rect& R1, Rect& R2);
    std::vector<DPoint>::iterator
    merge_mme(std::vector<DPoint>& v1, std::vector<DPoint>& v2,
              Pos sep, int o);
};

#endif
