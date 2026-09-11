// SPDX-License-Identifier: GPL-3.0-or-later
/**
 * @file draw_curve.h
 * @brief Draw a curve in an image
 * 
 * (C) 2011-2014, 2019, Pascal Monasse <pascal.monasse@enpc.fr>
 */

#ifndef DRAW_CURVE_H
#define DRAW_CURVE_H

#include "cc.h"

/// Inherit from this class to apply transform on the fly while drawing.
struct TransformPoint {
    virtual ~TransformPoint() {}
    virtual DPoint operator()(const DPoint& p) const { return p; }
};

template <typename T>
void draw_curve(const std::vector<DPoint>& curve, T v, T* im, int w, int h,
                const TransformPoint& t=TransformPoint());

#include "draw_curve.cpp"

#endif
