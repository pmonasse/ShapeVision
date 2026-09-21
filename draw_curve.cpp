// SPDX-License-Identifier: GPL-3.0-or-later
/**
 * @file draw_curve.cpp
 * @brief Draw a curve in an image
 * 
 * (C) 2011-2014, 2019, 2026 Pascal Monasse <pascal.monasse@enpc.fr>
 */

#ifdef DRAW_CURVE_H
#include <cmath>

/// Check point is inide image
static int inside(int x, int y, int w, int h) {
    return 0<=x && x<w && 0<=y && y<h;
}

/// Draw line in image
template <typename T>
void draw_line(const DPoint& p, const DPoint& q, T v, T* im, int w, int h) {
    int x0=(int)round(p.x), x1=(int)round(q.x);
    int y0=(int)round(p.y), y1=(int)round(q.y);
    if(x0==x1 && y0==y1) {
        if(inside(x0, y0, w, h))
            im[y0*w+x0] = v;
        return;
    }
    int sx = (x0<x1)? +1: -1;
    int sy = (y0<y1)? +1: -1;
    int dx=x1-x0, dy=y1-y0;
    int adx=sx*dx, ady=sy*dy;
    int x=0,y=0;
    if(adx>=ady) {
        int z=-adx/2;
        while(x!=dx) {
            if(inside(x+x0, y+y0, w, h))
                im[(y+y0)*w+(x+x0)] = v;
            x += sx;
            z += ady;
            if(z>0) {
                y += sy;
                z -= adx;
            }
        }
    } else {
        int z=-ady/2;
        while(y!=dy) {
            if(inside(x+x0, y+y0, w, h))
                im[(y+y0)*w+(x+x0)] = v;
            y += sy;
            z += adx;
            if(z>0) {
                x += sx;
                z -= ady;
            }
        }
    }
}

/// Draw curve in image
template <typename T>
void draw_curve(const std::vector<DPoint>& curve, T v, T* im, int w, int h,
                const TransformPoint& t) {
    if(curve.empty())
        return;
    std::vector<DPoint>::const_iterator it=curve.begin();
    DPoint o = *it++;
    while(it != curve.end()) {
        draw_line(t(o), t(*it), v, im,w,h);
        o = *it++;
    }
}

#endif
