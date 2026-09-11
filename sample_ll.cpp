// SPDX-License-Identifier: MPL-2.0
/**
 * @file sample_ll.cpp
 * @brief Sampling of level line from an image
 * 
 * (C) 2011-2016, 2019, 2026 Pascal Monasse <pascal.monasse@enpc.fr>
 */

#include "sample_ll.h"
#include <cmath>
#include <cassert>

/// Vector addition (Pos+Pos)
inline Pos operator+(Pos p1, Pos p2) {
    return Pos(p1.x+p2.x, p1.y+p2.y);
}
inline Pos& operator+=(Pos& p1, Pos p2) {
    p1.x+=p2.x; p1.y+=p2.y;
    return p1;
}

/// Vector addition (Pos+DPoint)
inline DPoint operator+(Pos p1, const DPoint& p2) {
    return DPoint(p1.x+p2.x, p1.y+p2.y);
}

/// Vector subtraction (DPoint-DPoint)
inline DPoint operator-(const DPoint& p1, const DPoint& p2) {
    return DPoint(p1.x-p2.x, p1.y-p2.y);
}

/// Vector multiplication (float*Pos)
inline DPoint operator*(float f, Pos p) {
    return DPoint(f*p.x, f*p.y);
}

/// Return x for y=v on line joining (0,v0) and (1,v1).
inline float lerp(float v0, float v, float v1) {
    return (v-v0)/(v1-v0);
}

/// Parameters of a hyperbola.
/// Inside the dual pixel, the level set has implicit equation
/// \f[ D*(x-xs)(y-ys)+N/D = l. \f]
/// This is true only if \f$D\neq0\f$ (otherwise we have just a line segment).
/// The center of the hyperbola (xs,ys) is a saddle point, its level is N/D.
/// Of interest, the vertex of the hyperbola is a point of maximal curvature.
/// It is located at
/// \f[(xs,ys)+(\pm\sqrt{|(Dl-N)/D^2|},\pm\sqrt{|(Dl-N)/D^2|}).\f]
/// The signs are determined by the fact that the vertex is in the same quadrant
/// as the input point p with respect to (xs,ys).
/// The equation of hyperbola is written
/// \f[ (x-xs)(y-ys) = \delta. \f]
class Hyperbola {
public:
    float num, denom; /// The saddle value is num/denom
    DPoint s; ///< Saddle point=center of hyperbola
    DPoint v; ///< Vertex of hyperbola=point of maximal curvature
    float delta; ///< Parameter of hyperbola (sqrt(2*delta) = semi major axis)

    Hyperbola(Pos pos, const DPoint& p, float lev[4], float l);
    bool valid() const { return (denom!=0); }
    bool vertex_in_dual_pixel(Pos p) const;
    void sample(const DPoint& p1, const DPoint& p2, int ptsPixel,
                std::vector<DPoint>& line) const;
private:
    static float sign(float f) { return (f>0)? +1: -1; }
};

/// Decompose hyperbola branch.
/// \param pos the top-left vertex of the dual pixel.
/// \param p a point on an edgel of the dual pixel and on the hyperbola.
/// \param level the levels at the four vertices of the dual pixel.
/// \param l level.
/// The hyperbola can be degenerate (a segment), in which case \c s, \c v and
/// \c delta make no sense. The method \c valid() must be used to check.
Hyperbola::Hyperbola(Pos pos, const DPoint& p, float level[4], float l) {
    num   = level[0]*level[2]-level[1]*level[3];
    denom = level[0]+level[2]-level[1]-level[3];
    delta = 0;
    if(denom == 0)
        return; // Degenerate hyperbola
    float d = 1.0f/denom;
    s.x = pos.x + (level[0]-level[3])*d;
    s.y = pos.y + (level[0]-level[1])*d;
    delta = (denom*l-num)*(d*d);
    d = sqrt(std::abs(delta));
    v.x = s.x + sign(p.x-s.x)*d;
    v.y = s.y + sign(p.y-s.y)*d;
}

/// Tell if the vertex of the hyperbola branch is inside the dual pixel of
/// top-left corner \a p.
bool Hyperbola::vertex_in_dual_pixel(Pos p) const {
    return valid() && (p.x<v.x && v.x<p.x+1 && p.y<v.y && v.y<p.y+1);
}

/// Sample branch of hyperbola from p1 to p2 of equation (x-xs)(y-ys)=delta:
/// \param p1 start point.
/// \param p2 end point.
/// \param ptsPixel number of points of discretization per pixel.
/// \param[out] line where the sampled points are stored.
void Hyperbola::sample(const DPoint& p1, const DPoint& p2, int ptsPixel,
                       std::vector<DPoint>& line) const {
    if(ptsPixel<2) return;
    DPoint p = p2-p1;
    if(p.x<0) p.x=-p.x;
    if(p.y<0) p.y=-p.y;
    if(p.x>p.y) { // Uniform sample along x
        int n = ceil(p.x*ptsPixel);
        float dx = (p2.x-p1.x)/n;
        p = p1;
        for(int i=1; i<n; i++) {
            p.x += dx;
            p.y = s.y + delta/(p.x-s.x);
            line.push_back(p);
        }
    } else { // Uniform sample along y
        int n = ceil(p.y*ptsPixel);
        float dy = (p2.y-p1.y)/n;
        p = p1;
        for(int i=1; i<n; i++) {
            p.y += dy;
            p.x = s.x + delta/(p.y-s.y);
            line.push_back(p);
        }
    }
}

/// Find side of \a mmeTo adjacent to \a mmeFrom.
int find_side_entry(const DPoint& mmeFrom, const DPoint& mmeTo, const CC& cc) {
    DPoint brFrom = cc.mme_br(mmeFrom), brTo = cc.mme_br(mmeTo);
    if(brFrom.y == mmeTo.y) return 0;
    if(mmeFrom.x == brTo.x) return 1;
    if(mmeFrom.y == brTo.y) return 2;
    if(brFrom.x == mmeTo.x) return 3;
    assert(false);
}

/// Find position of border point of level line on dual pixel at edgel \a side.
/// \param pos Top-left border of dual pixel.
/// \param side in [0,3].
/// \param level of level line.
/// \param[out] levels filled with values at the four corners of the dual pixel.
/// \param cc contours and continua
/// \param data image values.
DPoint border_point_ll(Pos pos, int side, float level, float levels[4],
                       const CC& cc, const float* data) {
    // p[i]+delta[i]=p[i+1] if p spans clockwise the corners of unit square
    static const Pos delta[] = {{+1,0}, {0,+1}, {-1,0}, {0,-1}};
    int i = cc.idx(pos);
    int idx[4] = {i, i+1, i+cc.w+1, i+cc.w};
    for(int j=0; j<4; j++)
        levels[j] = data[idx[j]];
    int sideNext = (side+1) & 3; // 4->0
    float t = lerp(levels[side], level, levels[sideNext]);
    assert(0<=t && t<=1);
    for(int d=0; d<side; d++) pos += delta[d];
    return pos + t*delta[side];
}

/// Extract level line in a continuum.
/// \param ctn the continuum.
/// \param level the value at level line, between inf- and sup-contour levels
/// \param cc the contours & continua structure containing \a ctn.
/// \param data the values of pixels in a 1D array.
/// \param ptsPixel number of points of discretization per pixel.
std::vector<DPoint> sample_ll(const Continuum& ctn, float level,
                              const CC& cc, const float* data,
                              int ptsPixel) {
    int side = ctn.sideIn;
    if(side<0)
        side = find_side_entry(ctn.mme.back(), ctn.mme.front(), cc);
    Pos pos((short)ctn.mme.front().x, (short)ctn.mme.front().y);
    float levels[4];
    DPoint p = border_point_ll(pos, side, level, levels, cc, data);
    std::vector<DPoint> ll;
    ll.push_back(p);
    for(size_t i=0, n=ctn.mme.size(); i!=n; i++) {
        int side=-1;
        if(i+1==n) {
            side = ctn.sideOut;
            if(side<0)
                side = find_side_entry(ctn.mme.front(), ctn.mme[i], cc);
        } else
            side = find_side_entry(ctn.mme[i+1], ctn.mme[i], cc);
        Pos pos((short)ctn.mme[i].x, (short)ctn.mme[i].y);
        DPoint q = border_point_ll(pos, side, level, levels, cc, data);
        // Compute hyperbola equation
        DPoint p = ll.back();
        Hyperbola h(pos, p, levels, level);
        if(h.valid() && ptsPixel>0) { // No sampling if not hyperbola (straight)
            bool vInside = h.vertex_in_dual_pixel(pos);
            if(std::abs(h.delta) < 1.0e-2f) { // Saddle: one or two segments
                if(vInside)
                    ll.push_back(h.v); // Put vertex only (almost saddle point)
            } else {
                if(vInside) { // Sample from entry point to vertex of hyperbola
                    h.sample(p, h.v, ptsPixel, ll);
                    ll.push_back(p=h.v);
                }
                h.sample(p, q, ptsPixel, ll); // Sample until end point
            }
        }
        ll.push_back(q);
    }
    return ll;
}
