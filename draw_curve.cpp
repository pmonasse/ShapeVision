// SPDX-License-Identifier: GPL-3.0-or-later
/**
 * @file draw_curve.cpp
 * @brief Draw a curve in an image
 * 
 * (C) 2011-2014, 2019, Pascal Monasse <pascal.monasse@enpc.fr>
 */

#ifdef DRAW_CURVE_H

/// max(0,min(abs(v),m-1))
static int clip(float v, int m) {
    if(v<0)
        return 0;
    if(v>=m)
        return m-1;
    return (int)v;
}

/// Draw line in image
template <typename T>
void draw_line(const DPoint& p, const DPoint& q, T v, T* im, int w, int h) {
    int x0=clip(p.x,w), x1=clip(q.x,w);
    int y0=clip(p.y,h), y1=clip(q.y,h);
    if(x0==x1 && y0==y1) {
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
