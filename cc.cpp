// SPDX-License-Identifier: MPL-2.0
/**
 * @file cc.cpp
 * @brief Contours & Continua
 * @author Pascal Monasse <pascal.monasse@enpc.fr>
 * @date 2025-2026
 */

#include "cc.h"
#include <algorithm>
#include <cassert>

DPoint pos2DPoint(Pos p) {
    return DPoint((double)p.x, (double)p.y);
}

DPoint min(const DPoint& p1, const DPoint& p2) {
    return DPoint(std::min(p1.x,p2.x), std::min(p1.y,p2.y));
}

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

/// Place v[i] at position order[i]. \a order is a permutation of {0,1,2,3}.
template <typename T>
void apply_permutation(const int order[4], T v[4]) {
    unsigned char todo = (1<<4)-1;
    for(int i=0; i<4; i++)
        if(todo & 1<<i) {
            for(int j=order[i]; j!=i; j=order[j]) {
                std::swap(v[j],v[i]);
                todo ^= 1<<j;
            }
            todo ^= 1<<i;
        }
}

/// Find the canonical contour and perform path compression.
int CC::root_contour(int i) {
    int j=contours[i].parent;
    if(j<0)
        return i;
    return (contours[i].parent = root_contour(j));
}

/// Set contour at index \a i2 have the same canonical element as at \a i1.
void CC::merge_contours(int i1, int i2) {
    assert(contours[i1].lvl == contours[i2].lvl);
    i1 = root_contour(i1);
    i2 = root_contour(i2);
    if(i1!=i2)
        contours[i2].parent = i1;
}

/// Create a continuum with indexes of the inf and sup contour.
/// Return an identifier (index in array) for the continuum.
int CC::create_continuum(Pos inf, Pos sup, const DPoint& p) {
    int i=(int)continua.size();
    int j=root_contour(inf), k=root_contour(sup);
    if(contours[j].lvl > contours[k].lvl)
        std::swap(j,k);
    Continuum c(j,k);
    c.mme.push_back(p);
    continua.push_back(c);
    return i;
}

/// Find the canonical contour and perform path compression.
int CC::root_continuum(int i) {
    int j=continua[i].parent;
    if(j<0)
        return i;
    return (continua[i].parent = root_continuum(j));
}

/// Create a virtual sample (saddle point) in dual pixel at p.
Pos CC::create_saddle(Pos p, const float lvl[4]) {
    Pos q(p.x, p.y+h);
    Contour& c = contours[idx(q)];
    c.p = pos2DPoint(p);
    float num=   lvl[0]*lvl[2] - lvl[1]*lvl[3];
    float denom=(lvl[0]+lvl[2])-(lvl[1]+lvl[3]);
    c.p.x += (lvl[0]-lvl[1])/denom;
    c.p.y += (lvl[0]-lvl[3])/denom;
    c.lvl = num/denom;
    // The code relies on the following properties, not guaranteed with float
    assert(c.p.x!=(int)c.p.x);
    assert(c.p.y!=(int)c.p.y);
    assert((lvl[0]-c.lvl)*(lvl[1]-c.lvl)<0);
    assert((lvl[1]-c.lvl)*(lvl[2]-c.lvl)<0);
    assert((lvl[2]-c.lvl)*(lvl[3]-c.lvl)<0);
    assert((lvl[3]-c.lvl)*(lvl[0]-c.lvl)<0);
    return q;
}

/// Fill the chain-code of an uninterrupted edge of an mme.
/// The vertices are in \a v, their level in \a lvl. The top-left corner is dtl.
/// Return the index of the newly created continuum or -1 if the vertices are
/// in a single contour.
int CC::fill_simple_chainCode(std::list<int>& L, const float lvl[2],
                              const Pos v[2], const DPoint& dtl) {
    int i=-1;
    if(lvl[0] == lvl[1])
        merge_contours(v[0],v[1]);
    else {
        i = create_continuum(v[0],v[1], dtl);
        L.push_back(i);
    }
    return i;
}

/// Constructor of rectangle of size 1x1, needing the four levels to build
/// the chain-codes. Warning: lvl is modified as a side-effect.
Rect CC::build_mme(Pos p, float lvl[4]) {
    int rank[4] = {0,1,2,3}, order[4];
    auto CompareValue = [lvl](int i, int j) { return lvl[i]<lvl[j]; };
    std::sort(rank, rank+4, CompareValue);
    for(int i=0; i<4; i++) order[rank[i]] = i; // inverse permutation
    Rect R(p, Pos(p.x+1,p.y+1));
    Pos v[] = {p, Pos(R.br.x,p.y), R.br, Pos(p.x,R.br.y)};
    apply_permutation(order, v);

    int c[4] = {-1,-1,-1,-1}; // Up to 4 continua
    if(((rank[0]+rank[1]) & 1) == 0) { // Smallest two diagonally opposite
        if(lvl[rank[1]] < lvl[rank[2]]) { // Saddle
            Pos s = create_saddle(p, lvl);
            int id=idx(s);
            DPoint ps = contours[id].p;
            for(int i=0; i<4; i++) {
                DPoint p = min(pos2DPoint(v[i]), ps);
                c[i] = create_continuum(v[i], s, p);
            }
            for(int i=0; i<=1; i++)
                for(int j=2; j<=3; j++) {
                    int eid = edge_id(rank[i],rank[j]);
                    R.chainCode[eid].push_back({c[i], c[j]});
                }
            return R;
        }
        std::swap(rank[1],rank[2]); // Make smallest two adjacent
        std::swap(v[1],v[2]);
    }

    apply_permutation(order, lvl);
    for(int i=0; i<4; i++)
        R.chainCode[i].push_back({});

    DPoint dtl = pos2DPoint(R.tl);
    for(int i=0; i<4; i+=2) { // Chain-codes for edges between min 2 and max 2
        int j = edge_id(rank[i], rank[i+1]);
        std::list<int>& L = R.chainCode[j].back();
        c[i] = fill_simple_chainCode(L, lvl+i, v+i, dtl);
    }

    if((rank[1]+rank[2]) & 1) { // two adjacent intermediate level vertices
        int eInt = edge_id(rank[1],rank[2]); // intermediate edge
        std::list<int>& Lint = R.chainCode[eInt].back();
        c[1] = fill_simple_chainCode(Lint, lvl+1, v+1, dtl);

        int eMm = (eInt+2)%4; // opposite edge, linking min and max
        std::list<int>& Lmm = R.chainCode[eMm].back();
        for(int i=0; i<3; i++)
            if(c[i]>=0)
                Lmm.push_back(c[i]);
    } else { // opposite intermediate level vertices
        if(lvl[1] == lvl[2])
            merge_contours(v[1],v[2]);
        else
            c[1] = create_continuum(v[1],v[2], dtl);

        int e02 = edge_id(rank[0],rank[2]);
        std::list<int>& L02 = R.chainCode[e02].back();
        if(lvl[0] == lvl[2])
            merge_contours(v[0],v[2]);
        else
            for(int i=0; i<2; i++)
                if(c[i]>=0)
                    L02.push_back(c[i]);

        int e13 = (e02+2)%4;
        std::list<int>& L13 = R.chainCode[e13].back();
        for(int i=1; i<3; i++)
            if(c[i]>=0)
                L13.push_back(c[i]);
    }
    return R;
}

/// Check whether dual pixel of top-left corner \a p is adjacent to edge of
/// top-left corner \a sep and orientation \a o (0=vertical, 1=horizontal).
bool CC::adjacent_rect(const DPoint& p, Pos sep, int o) const {
    int oo=1-o;
    if((int)p[oo] != sep[oo])
        return false;
    if((int)p[o]+1==sep[o] &&
       (p[o]!=(int)p[o] || contours[idx((int)p.x,(int)p.y+h)].p.x<0))
        return true; // p above or left of sep
    return ((int)p[o]==sep[o] && p[o]==(int)p[o]); // p below or right of sep
}

/// When two continua meeting along edge of top-left \a sep have mme
/// \a v1 and \a v2, append v2 to \a v1. They may have to be reordered so that
/// the edge is no longer a boundary. The orientation of the edge is given by
/// o (0=vertical, 1=horizontal).
/// Return iterator to the first element of junction.
std::vector<DPoint>::iterator
CC::merge_mme(std::vector<DPoint>& v1, std::vector<DPoint>& v2, Pos sep, int o){
    const DPoint& p = v1.front();
    if(adjacent_rect(p, sep, o))
        reverse(v1.begin(), v1.end());
    int n = v1.size();
    const DPoint& q = v2.back();
    if(adjacent_rect(q, sep, o))
        reverse(v2.begin(), v2.end());
    v1.insert(v1.end(), v2.begin(), v2.end());
    return v1.begin()+n;
}

/// Return bottom-right corner of mme whose top-left corner is \a p.
DPoint CC::mme_br(const DPoint& p) const {
    DPoint q = contours[idx((int)p.x,(int)p.y+h)].p;
    if(q.x<0 || p.x == q.x)
        q.x = (int)p.x+1;
    if(q.y<0 || p.y == q.y)
        q.y = (int)p.y+1;
    return q;
}

/// Find in chainCode \a L the continuum of index \a iSplit, that must be
/// present. Insert before the continuum \a iCtn.
/// Used during propagation of continua and contour when splitting a continuum.
void CC::insert_chainCode(std::list<int>& L, int iSplit, int iCtn) {
    assert(iSplit==root_continuum(iSplit));
    std::list<int>::iterator it=L.begin(), end=L.end();
    for(; it!=end; ++it)
        if((*it=root_continuum(*it))==iSplit)
            break;
    assert(it!=L.end());
    L.insert(it, iCtn);
}

/// If the chainCode \a L contains the continuum \a iSplit, insert \a iCtn.
/// \a iSplit must be a root continuum. \a v is a contour value that is
/// within the interval defined by contours bounding \a iSplit.
/// Return whether it did something.
/// TODO: instead of linear search by insert_chainCode, use dichotomy.
bool CC::mark_exit_side(std::list<int>& L, float v, int iSplit, int iCtn) {
    if(! L.empty()) {
        float v1=contours[continua[L.front()].infCtr].lvl;
        float v2=contours[continua[L.back()] .supCtr].lvl;
        if((v1-v)*(v2-v)<0) {
            insert_chainCode(L, iSplit, iCtn);
            return true;
        }
    }
    return false;
}

/// Mark continuum \a iCtn and its sup-contour crossing the continuum of index
/// \a iSplit. This is for the exit edge of the last mme of \a iSplit out of
/// \a R. The side \a iSideIn (0..3), representing entry edge, must be skipped
/// from the search of exit edge.
void CC::mark_exit(Rect& R, int iSplit, int iCtn, int iSideIn) {
    const DPoint& p = continua[iSplit].mme.back();
    const int j = continua[iCtn].supCtr;
    const float v = contours[j].lvl;
    if(iSideIn != 0 && p.y == R.tl.y) { // Upper edge
        std::list<std::list<int>>::iterator i = R.chainCode[0].begin();
        std::advance(i, (int)p.x-R.tl.x);
        if( mark_exit_side(*i, v, iSplit, iCtn) )
            return;
    }
    if(iSideIn != 3 && p.x == R.tl.x) { // Left edge
        std::list<std::list<int>>::iterator i = R.chainCode[3].begin();
        std::advance(i, (int)p.y-R.tl.y);
        if( mark_exit_side(*i, v, iSplit, iCtn) )
            return;
    }
    DPoint q = mme_br(p);
    if(iSideIn != 1 && q.x == R.br.x) { // Right edge
        std::list<std::list<int>>::iterator i = R.chainCode[1].begin();
        std::advance(i, (int)p.y-R.tl.y);
        if( mark_exit_side(*i, v, iSplit, iCtn) )
            return;
    }
    if(iSideIn != 2 && q.y == R.br.y) { // Bottom edge
        std::list<std::list<int>>::iterator i = R.chainCode[2].begin();
        std::advance(i, (int)p.x-R.tl.x);
        if( mark_exit_side(*i, v, iSplit, iCtn) )
            return;
    }
    assert(false);
}

/// Check if m<n=i or n<m=i.
bool inside(int i, int m, int n) {
    if(m>n)
       std::swap(m,n);
    return (m<i && i==n);
}

/// Given two adjacent mme, return the side of edge of \a dst that
/// was crossed when coming from \a src.
int find_side_entry(const DPoint& src, const DPoint& dst) {
    if((int)src.x!=(int)dst.x)
    return 2+((int)dst.x-(int)src.x);
  return 1-((int)dst.y-(int)src.y);
}

/// The continuum of index \a iSplit must be split by the sup-contour of
/// continuum \a iCtn. The side \a iSideIn (0..3) is the entry direction.
/// \a it points to the mme of entry.
/// All crossings through edges at the same level as \a sep are recorded in
/// the chain codes.
void CC::split_continuum(Rect& Rsrc, Rect& Rdst,
                         std::vector<DPoint>::iterator it, Pos sep,
                         int iSplit, int iCtn, int iSideIn) {
    const DPoint& p = *it;
    const int dir = iSideIn&1; // adjacency of rects: 0=horizontal, 1=vertical
    std::list<std::list<int>>::iterator i = Rdst.chainCode[iSideIn].begin();
    std::advance(i, (int)p[dir]-Rdst.tl[dir]);
    insert_chainCode(*i, iSplit, iCtn);
    Rect* R[2] = {&Rsrc, &Rdst};
    int ori[2] = {(iSideIn+2)%4, iSideIn};
    int side=1;
    std::vector<DPoint>::iterator itn=std::next(it),
                                  end=continua[iCtn].mme.end();
    const int dim=1-dir, lim=sep[dim];
    for(; itn!=end; it=itn++)
        if(inside(lim, (int)(*it)[dim], (int)(*itn)[dim])) { // Crossing
           i = R[side]->chainCode[ori[side]].begin();
           std::advance(i, (int)(*it)[dir]-Rsrc.tl[dir]);
           insert_chainCode(*i, iSplit, iCtn);
           side = 1-side;
           i = R[side]->chainCode[ori[side]].begin();
           std::advance(i, (int)(*it)[dir]-Rsrc.tl[dir]);
           insert_chainCode(*i, iSplit, iCtn);
        }
    iSideIn = find_side_entry(*std::prev(it), *it);
    mark_exit(*R[side], iSplit, iCtn, iSideIn);
    continua[iSplit].infCtr = continua[iCtn].supCtr;
}

/// Given two chain-codes \a L1 and \a L2 along a common edge, merge or split
/// continua, merge contours. The vertical common edge has top-left endpoint
/// at \a sep and orientation \a o (1=horizontal, 0=vertical). The
/// enclosing rectangles \a R1 and \a R2 are adjacent.
void CC::propagate(Rect& R1, Rect& R2, Pos sep, int o,
                   const std::list<int>& L1, const std::list<int>& L2) {
    std::list<int>::const_iterator i1=L1.begin(), i2=L2.begin();
    if(i1 == L1.end()) { // Single contour
        assert(i2 == L2.end());
        return;
    }
    std::vector<DPoint>::iterator it;
    int ic1, ic2, j1, j2; float l1, l2;
    bool inc1=true, inc2=true;
    do {
        if(inc1) {
            assert(i1!=L1.end());
            ic1 = root_continuum(*i1++);
            j1 = continua[ic1].supCtr;
            l1 = contours[j1].lvl;
        }
        if(inc2) {
            assert(i2!=L2.end());
            ic2 = root_continuum(*i2++);
            j2 = continua[ic2].supCtr;
            l2 = contours[j2].lvl;
        }
        inc1 = inc2 = true;
        if(l1<l2) { // split continuum ic2
            it = merge_mme(continua[ic1].mme, continua[ic2].mme, sep,o);
            split_continuum(R1, R2, it, sep, ic2, ic1, (o+3)%4);
            inc2 = false;
        } else if(l2<l1) { // split continuum ic1
            it = merge_mme(continua[ic2].mme, continua[ic1].mme, sep,o);
            split_continuum(R2, R1, it, sep, ic1, ic2, o+1);
            inc1 = false;
        } else { // l1==l2
            merge_contours(j1,j2);
            if(ic1 != ic2) {
                merge_mme(continua[ic1].mme, continua[ic2].mme, sep,o);
                continua[ic2].parent = ic1;
                continua[ic2].mme.clear();
                continua[ic2].mme.shrink_to_fit();
            }
        }
    } while(i1!=L1.end() || i2!=L2.end());
}

/// Merge two adjacent rectangles.
Rect CC::merge_rectangles(Rect& R1, Rect& R2) {
    int o = -1; // Relative orientation of R1 and R2. 0,1=horizontal,vertical
    if(R1.tl.x == R2.tl.x)
        o=1; // Vertical neighbors, horizontal edges
    if(R1.tl.y == R2.tl.y)
        o=0; // Horizontal neighbors, vertical edges
    assert(o==0 || o==1);
    int o1=o+1, o2=(o1+2)%4;

    // Propagate chain-codes along common edges
    assert(R1.chainCode[o1].size()==R2.chainCode[o2].size());
    std::list<std::list<int>>::const_iterator i1=R1.chainCode[o1].begin(),
                                              i2=R2.chainCode[o2].begin(),
                                              end=R1.chainCode[o1].end();
    Pos sep = R2.tl;
    for(; i1!=end; ++i1, ++i2, ++sep[1-o])
        propagate(R1, R2, sep, o, *i1, *i2);

    // Move chain-codes at frame of R
    Rect R(R1.tl, R2.br);
    std::swap(R1.chainCode[o2],R.chainCode[o2]);
    std::swap(R2.chainCode[o1],R.chainCode[o1]);
    std::swap(R1.chainCode[o],R.chainCode[o]);
    R.chainCode[o].splice(R.chainCode[o].end(),R2.chainCode[o]);
    o += 2;
    std::swap(R1.chainCode[o],R.chainCode[o]);
    R.chainCode[o].splice(R.chainCode[o].end(),R2.chainCode[o]);
    return R;
}

/// Constructor with image.
CC::CC(const float* im, int w, int h): R(Pos(0,0),Pos(w-1,h-1)), w(w), h(h) {
    contours = new Contour[2*w*h]; // 2x due to virtual samples
    for(int i=0,idx=0; i<h; i++)
        for(int j=0; j<w; j++,idx++) {
            contours[idx].p = DPoint(j,i);
            contours[idx].lvl = im[idx];
        }
    std::vector<Rect> rects;
    for(int i=0; i+1<h; i++)
        for(int j=0; j+1<w; j++) {
            int idx = i*w+j;
            float lvl[4] = { im[idx], im[idx+1], im[idx+1+w], im[idx+w] };
            rects.push_back( build_mme(Pos(j,i),lvl) );
        }
    // C&C propagation
    int w2=w-1, h2=h-1;
    while(w2>1 || h2>1) {
        // Horizontal propagation
        std::vector<Rect> res;
        res.reserve((w2+1)/2*h2);
        for(int i=0; i<h2; i++) {
            for(int j=0; j+1<w2; j+=2) {
                Rect r = merge_rectangles(rects[i*w2+j], rects[i*w2+j+1]);
                res.push_back( std::move(r) );
            }
            if(w2&1)
                res.push_back( std::move(rects[i*w2+w2-1]) );
        }
        std::swap(rects,res); res.clear();
        w2=(w2+1)/2;
        // Vertical propagation
        res.reserve((h2+1)/2*w2);
        for(int i=0; i+1<h2; i+=2) {
            for(int j=0; j<w2; j++) {
                Rect r = merge_rectangles(rects[i*w2+j], rects[(i+1)*w2+j]);
                res.push_back( std::move(r) );
            }
        }
        if(h2&1)
            for(int j=0; j<w2; j++)
                res.push_back( std::move(rects[(h2-1)*w2+j]) );
        std::swap(rects,res); res.clear();
        h2=(h2+1)/2;
    }
    std::swap(R, rects.back());
}
