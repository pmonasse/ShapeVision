#include "cc.h"
#include <list>
#include <algorithm>
#include <cassert>

DPoint min(const DPoint& p1, const DPoint& p2) {
    return DPoint(std::min(p1.x,p2.x), std::min(p1.y,p2.y));
}

struct Rect {
    Pos tl, br; ///< Top-left and bottom-right corners of rectangle
    std::list<std::list<int>> chainCode[4];
    Rect(CC& cc, Pos p, float lvl[4]);
};

struct CompareValue {
    const float* lvl;
    CompareValue(const float levels[4]): lvl(levels) {}
    bool operator()(int i, int j) const { return lvl[i]<lvl[j]; }
};

/// Given i and j the index of two consecutive vertices of a square, compute
/// the edge index. The indices are as follows:
/// 0  0  1
///  +---+
/// 3|   |1
///  +---+
/// 3  2  2
int edge_id(int i, int j) {
    assert((i+j)&1); // Make sure the index are consecutive
    int k = std::min(i,j);
    if(k==0 && std::max(i,j)==3)
        k = 3;
    return k;
}

/// Constructor of rectangle of size 1x1, needing the four levels to build
/// the chain-codes.
Rect::Rect(CC& cc, Pos p, float lvl[4]) : tl(p), br(p.x+1,p.y+1) {
    const Pos v[] = {tl, Pos(br.x,tl.y), br, Pos(tl.x,br.y)};
    int rank[4] = {0,1,2,3};
    std::sort(rank, rank+4, CompareValue(lvl));
    Pos vo[4]; // Vertices ordered by level
    for(int i=0; i<4; i++)
        vo[i] = v[rank[i]];
    int c[4] = {-1,-1,-1,-1}; // Up to 4 continua
    if(((rank[0]+rank[1])&1) == 0) { // Smallest two diagonally opposite
        if(lvl[rank[1]] < lvl[rank[2]]) { // Saddle
            Pos s=cc.create_saddle(p, lvl);
            int idx=cc.idx(s);
            const Contour& ctr = cc.contours[idx];
            for(int i=0; i<4; i++) {
                DPoint p = min(vo[i],ctr.p);
                c[i] = cc.create_continuum(vo[i],s, p);
            }
            for(int i=0; i<=1; i++)
                for(int j=2; j<=3; j++) {
                    int eid = edge_id(rank[i],rank[j]);
                    chainCode[eid].push_back(
                     std::list<int>{cc.idx(vo[i]),c[i],idx,c[j],cc.idx(vo[j])});
                }
            return;
        }
        std::swap(rank[1],rank[2]); // Make two smallest adjacent
        std::swap(vo[1],vo[2]);
    }

    for(int i=0; i<4; i++) {
        std::list<int> L;
        chainCode[i].emplace_back(L);
    }
    
    int eMin = edge_id(rank[0], rank[1]);
    chainCode[eMin].back().push_back(cc.idx(vo[0]));
    if(lvl[rank[0]]==lvl[rank[1]])
        cc.merge_contours(vo[0],vo[1]);
    else {
        c[0] = cc.create_continuum(vo[0],vo[1], tl);
        chainCode[eMin].back().push_back(c[0]);
        chainCode[eMin].back().push_back(cc.idx(vo[1]));
    }

    int eMax = edge_id(rank[2], rank[3]);
    chainCode[eMax].back().push_back(cc.idx(vo[2]));
    if(lvl[rank[2]]==lvl[rank[3]])
        cc.merge_contours(vo[2],vo[3]);
    else {
        c[1] = cc.create_continuum(vo[2],vo[3], tl);
        chainCode[eMax].back().push_back(c[1]);
        chainCode[eMax].back().push_back(cc.idx(vo[3]));
    }

    if((rank[1]+rank[2])&1) { // two adjacent intermediate level vertices
        int eInt = edge_id(rank[1],rank[2]); // intermediate edge
        chainCode[eInt].back().push_back(cc.idx(vo[1]));
        if(lvl[rank[1]]==lvl[rank[2]])
            cc.merge_contours(vo[1],vo[2]);
        else {
            c[2] = cc.create_continuum(vo[1],vo[2], tl);
            chainCode[eInt].back().push_back(c[2]);
            chainCode[eInt].back().push_back(cc.idx(vo[2]));
        }
        int eMm = (eInt+2)%4; // opposite edge, linking min and max
        chainCode[eMm].back().push_back(cc.idx(vo[0]));
        if(c[0]>=0) {
            chainCode[eMm].back().push_back(c[0]);
            chainCode[eMm].back().push_back(cc.idx(vo[1]));
        }
        if(c[2]>=0) {
            chainCode[eMm].back().push_back(c[2]);
            chainCode[eMm].back().push_back(cc.idx(vo[2]));
        }
        if(c[1]>=0) {
            chainCode[eMm].back().push_back(c[1]);
            chainCode[eMm].back().push_back(cc.idx(vo[3]));
        }
    } else { // opposite intermediate level vertices
        if(lvl[rank[1]] == lvl[rank[2]])
            cc.merge_contours(vo[1],vo[2]);
        else
            c[2] = cc.create_continuum(vo[1],vo[2], tl);
        int e02 = edge_id(rank[0],rank[2]);
        chainCode[e02].back().push_back(cc.idx(vo[0]));
        if(lvl[rank[0]] == lvl[rank[2]])
            cc.merge_contours(vo[0],vo[2]);
        else {
            if(c[0]>=0) {
                chainCode[e02].back().push_back(c[0]);
                chainCode[e02].back().push_back(cc.idx(vo[1]));
            }
            if(c[2]>=0) {
                chainCode[e02].back().push_back(c[2]);
                chainCode[e02].back().push_back(cc.idx(vo[2]));
            }
        }
        int e13 = (e02+2)%4;
        chainCode[e13].back().push_back(cc.idx(vo[1]));
        if(c[2]>=0) {
            chainCode[e13].back().push_back(c[2]);
            chainCode[e13].back().push_back(cc.idx(vo[2]));
        }
        if(c[1]>=0) {
            chainCode[e13].back().push_back(c[1]);
            chainCode[e13].back().push_back(cc.idx(vo[3]));
        }
    }
}

/// Constructor with image.
CC::CC(const float* im, int w, int h): w(w), h(h) {
    contours = new Contour[2*w*h]; // 2x due to virtual samples
    for(int i=0,idx=0; i<h; i++)
        for(int j=0; j<w; j++,idx++) {
            contours[idx].p = DPoint(j,i);
            contours[idx].lvl = im[idx];
        }
    std::vector<Rect> R;
    for(int i=0; i+1<h; i++)
        for(int j=0; j+1<w; j++) {
            int idx = i*w+j;
            float lvl[4] = { im[idx], im[idx+1], im[idx+1+w], im[idx+w] };
            R.push_back(Rect(*this,Pos(j,i),lvl));
        }
}

/// Create a virtual sample (saddle point) in dual pixel at p.
Pos CC::create_saddle(Pos p, float lvl[4]) {
    p.y += h;
    int i = idx(p);
    Contour& c = contours[i];
    float num=   lvl[0]*lvl[2] - lvl[1]+lvl[3];
    float denom=(lvl[0]+lvl[2])-(lvl[1]+lvl[3]);
    c.p.x = (lvl[0]-lvl[1])/denom;
    c.p.y = (lvl[0]-lvl[3])/denom;
    c.lvl = num/denom;
    return p;
}

/// Create a continuum with indexes of the inf and sup contour.
/// Return an identifier (index in array) for the continuum.
int CC::create_continuum(Pos inf, Pos sup, const DPoint& p) {
    int i=(int)continua.size();
    int j=idx(inf), k=idx(sup);
    if(contours[j].lvl > contours[k].lvl)
        std::swap(j,k);
    Continuum c(j,k);
    c.mme.push_back(p);
    continua.push_back(c);
    return i;
}

/// Set contour at c2 have the same canonical element as the one at c1.
void CC::merge_contours(Pos c1, Pos c2) {
    assert(contours[idx(c1)].lvl==contours[idx(c2)].lvl);
    c1 = root_contour(c1);
    c2 = root_contour(c2);
    if(c1!=c2)
        contours[idx(c2)].parent = c1;
}

/// Find the canonical contour and perform path compression.
Pos CC::root_contour(Pos c) {
    Pos p=contours[idx(c)].parent;
    if(p.x<0)
        return c;
    p = contours[idx(c)].parent = root_contour(p);
    return p;
}
