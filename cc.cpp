#include "cc.h"
#include <list>
#include <algorithm>
#include <cassert>

DPoint pos2DPoint(Pos p) {
    return DPoint((double)p.x, (double)p.y);
}

DPoint min(const DPoint& p1, const DPoint& p2) {
    return DPoint(std::min(p1.x,p2.x), std::min(p1.y,p2.y));
}

struct Rect {
    Pos tl, br; ///< Top-left and bottom-right corners of rectangle
    std::list<std::list<int>> chainCode[4];
    Rect(Pos topLeft, Pos bottomRight);
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

Rect::Rect(Pos topLeft, Pos bottomRight) : tl(topLeft), br(bottomRight) {}

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
            Contour& ctr = cc.contours[idx];
            ctr.p.x += p.x; ctr.p.y += p.y;
            for(int i=0; i<4; i++) {
                DPoint p = min(pos2DPoint(vo[i]),ctr.p);
                c[i] = cc.create_continuum(vo[i], s, p);
            }
            for(int i=0; i<=1; i++)
                for(int j=2; j<=3; j++) {
                    int eid = edge_id(rank[i],rank[j]);
                    chainCode[eid].push_back(
                     std::list<int>{cc.root_contour(vo[i]),c[i],idx,c[j],cc.root_contour(vo[j])});
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

    DPoint dtl = pos2DPoint(tl);
    int eMin = edge_id(rank[0], rank[1]);
    chainCode[eMin].back().push_back(cc.root_contour(vo[0]));
    if(lvl[rank[0]]==lvl[rank[1]])
        cc.merge_contours(vo[0],vo[1]);
    else {
        c[0] = cc.create_continuum(vo[0],vo[1], dtl);
        chainCode[eMin].back().push_back(c[0]);
        chainCode[eMin].back().push_back(cc.root_contour(vo[1]));
    }

    int eMax = edge_id(rank[2], rank[3]);
    chainCode[eMax].back().push_back(cc.root_contour(vo[2]));
    if(lvl[rank[2]]==lvl[rank[3]])
        cc.merge_contours(vo[2],vo[3]);
    else {
        c[1] = cc.create_continuum(vo[2],vo[3], dtl);
        chainCode[eMax].back().push_back(c[1]);
        chainCode[eMax].back().push_back(cc.root_contour(vo[3]));
    }

    if((rank[1]+rank[2])&1) { // two adjacent intermediate level vertices
        int eInt = edge_id(rank[1],rank[2]); // intermediate edge
        chainCode[eInt].back().push_back(cc.root_contour(vo[1]));
        if(lvl[rank[1]]==lvl[rank[2]])
            cc.merge_contours(vo[1],vo[2]);
        else {
            c[2] = cc.create_continuum(vo[1],vo[2], dtl);
            chainCode[eInt].back().push_back(c[2]);
            chainCode[eInt].back().push_back(cc.root_contour(vo[2]));
        }
        int eMm = (eInt+2)%4; // opposite edge, linking min and max
        chainCode[eMm].back().push_back(cc.root_contour(vo[0]));
        if(c[0]>=0) {
            chainCode[eMm].back().push_back(c[0]);
            chainCode[eMm].back().push_back(cc.root_contour(vo[1]));
        }
        if(c[2]>=0) {
            chainCode[eMm].back().push_back(c[2]);
            chainCode[eMm].back().push_back(cc.root_contour(vo[2]));
        }
        if(c[1]>=0) {
            chainCode[eMm].back().push_back(c[1]);
            chainCode[eMm].back().push_back(cc.root_contour(vo[3]));
        }
    } else { // opposite intermediate level vertices
        if(lvl[rank[1]] == lvl[rank[2]])
            cc.merge_contours(vo[1],vo[2]);
        else
            c[2] = cc.create_continuum(vo[1],vo[2], dtl);
        int e02 = edge_id(rank[0],rank[2]);
        chainCode[e02].back().push_back(cc.root_contour(vo[0]));
        if(lvl[rank[0]] == lvl[rank[2]])
            cc.merge_contours(vo[0],vo[2]);
        else {
            if(c[0]>=0) {
                chainCode[e02].back().push_back(c[0]);
                chainCode[e02].back().push_back(cc.root_contour(vo[1]));
            }
            if(c[2]>=0) {
                chainCode[e02].back().push_back(c[2]);
                chainCode[e02].back().push_back(cc.root_contour(vo[2]));
            }
        }
        int e13 = (e02+2)%4;
        chainCode[e13].back().push_back(cc.root_contour(vo[1]));
        if(c[2]>=0) {
            chainCode[e13].back().push_back(c[2]);
            chainCode[e13].back().push_back(cc.root_contour(vo[2]));
        }
        if(c[1]>=0) {
            chainCode[e13].back().push_back(c[1]);
            chainCode[e13].back().push_back(cc.root_contour(vo[3]));
        }
    }
}

/// Find in chainCode \a L the continuum of index \a iSplit, that should be
/// present at most once. Insert before the continuum \a iCtn and the contour
/// \a iCtr. This is used during propagation of continua and contour when
/// splitting a continuum.
bool insert_chainCode(CC& cc, std::list<int>& L, int iSplit, int iCtn, int iCtr) {
    assert(L.size()&1);
    std::list<int>::iterator it = L.begin();
    for(++it; it!=L.end(); advance(it,2)) {
        *it = cc.root_continuum(*it);
        if(*it == iSplit) {
            L.insert(it, iCtn);
            L.insert(it, iCtr);
            return true;
        }
    }
    return false;
}

/// Mark continuum \a iCtn and contour \a iCtr crossing the continuum of index
/// \a iSplit. This is for the exit edge of the last mme of \a iSplit out of
/// \a R. The side \a iSideIn (0..3), representing entry edge, must be skipped
/// from the search of exit edge.
void mark_exit(CC& cc, Rect& R, int iSplit, int iCtn, int iCtr, int iSideIn) {
    const DPoint& p = cc.continua[iSplit].mme.back();
    if(iSideIn != 0 && p.y == R.tl.y) { // Upper edge
        std::list<std::list<int>>::iterator i = R.chainCode[0].begin();
        std::advance(i, (int)p.x-R.tl.x);
        if( insert_chainCode(cc, *i, iSplit, iCtn, iCtr) )
            return;
    }
    if(iSideIn != 3 && p.x == R.tl.x) { // Left edge
        std::list<std::list<int>>::iterator i = R.chainCode[3].begin();
        std::advance(i, (int)p.y-R.tl.y);
        if( insert_chainCode(cc, *i, iSplit, iCtn, iCtr) )
            return;
    }
    DPoint q = cc.mme_br(p);
    if(iSideIn != 1 && q.x == R.br.x) { // Right edge
        std::list<std::list<int>>::iterator i = R.chainCode[1].begin();
        std::advance(i, (int)p.y-R.tl.y);
        if( insert_chainCode(cc, *i, iSplit, iCtn, iCtr) )
            return;
    }
    if(iSideIn != 2 && q.y == R.br.y) { // Bottom edge
        std::list<std::list<int>>::iterator i = R.chainCode[2].begin();
        std::advance(i, (int)p.x-R.tl.x);
        if( insert_chainCode(cc, *i, iSplit, iCtn, iCtr) )
            return;
    }
    assert(false);
}

/// Check if m<n=i or n<m=i.
bool inside(short int i, short int m, short int n) {
    if(m>n)
       std::swap(m,n);
    return (m<i && i==n);
}

/// Given two adjacent mme, return the side of edge of \a dst that
/// was crossed when coming from \a src.
int find_side_entry(const DPoint& src, const DPoint& dst) {
  if(src.x!=dst.x)
    return 2+((int)dst.x-(int)src.x);
  return 1-((int)dst.y-(int)src.y);
}

/// The continuum of index \a iSplit must be split by the continuum \a iCtn
/// with delimiting contour \a iCtr. The side \a iSideIn (0..3) is the entry
/// direction. \a it points to the mme of entry.
/// All crossings through edges at the same level as \a sep are recorded in
/// the chain codes.
void split_continuum(CC& cc, Rect& Rsrc, Rect& Rdst,
                     std::vector<DPoint>::iterator it, Pos sep,
                     int iSplit, int iCtn, int iCtr, int iSideIn) {
    cc.continua[iSplit].infCtr = iCtr;
    const DPoint& p = *it;
    const int dir = iSideIn&1; // adjacency of rects: 0=horizontal, 1=vertical
    std::list<std::list<int>>::iterator i = Rdst.chainCode[iSideIn].begin();
    std::advance(i, (int)p[dir]-Rdst.tl[dir]);
    bool b = insert_chainCode(cc, *i, iSplit, iCtn, iCtr);
    (void)b; assert(b);
    Rect* R[2] = {&Rsrc, &Rdst};
    int ori[2] = {(iSideIn+2)%4, iSideIn};
    int side=1;
    std::vector<DPoint>::iterator itn=std::next(it),
      end=cc.continua[iCtn].mme.end();
    int dim=1-dir, lim=sep[dim];
    for(; itn!=end; it=itn++)
        if(inside(lim, (*it)[dim], (*itn)[dim])) { // Crossing
           i = R[side]->chainCode[ori[side]].begin();
           std::advance(i, (int)(*it)[dir]-Rsrc.tl[dir]);
           bool b = insert_chainCode(cc, *i, iSplit, iCtn, iCtr);
           (void)b; assert(b);
           side = 1-side;
           i = R[side]->chainCode[ori[side]].begin();
           std::advance(i, (int)(*it)[dir]-Rsrc.tl[dir]);
           b = insert_chainCode(cc, *i, iSplit, iCtn, iCtr);
           (void)b; assert(b);
        }
    iSideIn = find_side_entry(*std::prev(it), *it);
    mark_exit(cc, *R[side], iSplit, iCtn, iCtr, iSideIn);
}

/// Given two chain-codes \a L1 and \a L2 along a common edge, merge or split
/// continua, merge contours. The vertical common edge has top-left endpoint
/// at \a sep and orientation \a o (1=horizontal, 0=vertical). The
/// enclosing rectangles \a R1 and \a R2 are adjacent.
void propagate(CC& cc, Rect& R1, Rect& R2, Pos sep, int o,
               const std::list<int>& L1, const std::list<int>& L2) {
    assert(!L1.empty() && !L2.empty());
    std::list<int>::const_iterator i1=L1.begin(), i2=L2.begin();
    assert(*i1 == *i2);
    ++i1; ++i2;
    if(i1 == L1.end()) { // Single contour
        assert(i2 == L2.end());
        return;
    }
    std::vector<DPoint>::iterator it;
    int ic1 = cc.root_continuum(*i1++);
    int ic2 = cc.root_continuum(*i2++);
    int j1 = cc.root_contour(*i1++);
    int j2 = cc.root_contour(*i2++);
    float l1 = cc.contours[j1].lvl;
    float l2 = cc.contours[j2].lvl;
    do {
        if(l1 == l2) {
            if(j1 != j2)
                cc.contours[j2].parent = j1;
            if(ic1 != ic2) {
                cc.merge_mme(cc.continua[ic1].mme, cc.continua[ic2].mme, sep,o);
                cc.continua[ic2].parent = ic1;
                cc.continua[ic2].mme.clear();
                cc.continua[ic2].mme.shrink_to_fit();
            }
        } else if(l1<l2) { // split continuum ic2
            it= cc.merge_mme(cc.continua[ic1].mme, cc.continua[ic2].mme, sep,o);
            split_continuum(cc, R1, R2, it, sep, ic2, ic1, j1, (o+3)%4);
        } else if(l2<l1) { // split continuum ic1
            it= cc.merge_mme(cc.continua[ic2].mme, cc.continua[ic1].mme, sep,o);
            split_continuum(cc, R2, R1, it, sep, ic1, ic2, j2, o+1);
        }
        float l1old=l1;
        if(l1old <= l2) {
            if(i1!=L1.end()) {
                ic1 = cc.root_continuum(*i1++);
                assert(i1!=L1.end());
                j1 = cc.root_contour(*i1++);
                l1 = cc.contours[j1].lvl;
            }
        }
        if(l1old >= l2) {
            if(i2!=L2.end()) {
                ic2 = cc.root_continuum(*i2++);
                assert(i2!=L2.end());
                j2 = cc.root_contour(*i2++);
                l2 = cc.contours[j2].lvl;
            }
        }
    } while(i1!=L1.end() || i2!=L2.end());
    ic1 = cc.root_continuum(ic1);
    ic2 = cc.root_continuum(ic2);
    if(ic1!=ic2) {
        cc.merge_mme(cc.continua[ic1].mme, cc.continua[ic2].mme, sep,o);
        cc.continua[ic2].parent = ic1;
        cc.continua[ic2].mme.clear();
        cc.continua[ic2].mme.shrink_to_fit();
    }
}

/// Merge two adjacent rectangles, separated by vertical edges.
Rect merge_rectangles(CC& cc, Rect& R1, Rect& R2) {
    int o = -1; // Relative orientation of R1 and R2. 0,1=horizontal,vertical
    if(R1.tl.x == R2.tl.x)
        o=1; // Vertical neighbors, horizontal edges
    if(R1.tl.y == R2.tl.y)
        o=0; // Horizontal edges, vertical neighbors
    assert(o==0 || o==1);
    int o1=o+1, o2=(o1+2)%4;

    // Propagate chain-codes along common edges
    assert(R1.chainCode[o1].size()==R2.chainCode[o2].size());
    std::list<std::list<int>>::const_iterator i1=R1.chainCode[o1].begin(),
                                              i2=R2.chainCode[o2].begin(),
                                              end=R1.chainCode[o1].end();
    Pos sep = R2.tl;
    for(; i1!=end; ++i1, ++i2, ++sep[1-o])
        propagate(cc, R1, R2, sep, o, *i1, *i2);

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
    // C&C propagation
    int w2=w-1, h2=h-1;
    while(w2>1 || h2>1) {
        // Horizontal propagation
        size_t n=R.size();
        for(int i=0; i<h2; i++) {
            for(int j=0; j+1<w2; j+=2) {
                Rect r = merge_rectangles(*this, R[i*w2+j], R[i*w2+j+1]);
                R.push_back(r);
            }
            if(w2&1)
                R.push_back(R[i*w2+w2-1]);
        }
        R.erase(R.begin(), R.begin()+n);
        w2=(w2+1)/2;
        // Vertical propagation
        n = R.size();
        for(int i=0; i+1<h2; i+=2) {
            for(int j=0; j<w2; j++) {
                Rect r = merge_rectangles(*this, R[i*w2+j], R[(i+1)*w2+j]);
                R.push_back(r);
            }
        }
        if(h2&1)
            for(int j=0; j<w2; j++)
                R.push_back(R[(h2-1)*w2+j]);
        R.erase(R.begin(), R.begin()+n);
        h2=(h2+1)/2;
    }
}

/// Return bottom-right corner of mme whose top-left corner is \a p.
DPoint CC::mme_br(const DPoint& p) const {
    DPoint q = contours[idx(Pos((int)p.x,(int)p.y+h))].p;
    if(q.x<0 || p.x == q.x)
        q.x = (int)p.x+1;
    if(q.y<0 || p.y == q.y)
        q.y = (int)p.y+1;
    return q;
}

/// Create a virtual sample (saddle point) in dual pixel at p.
Pos CC::create_saddle(Pos p, float lvl[4]) {
    p.y += h;
    int i = idx(p);
    Contour& c = contours[i];
    float num=   lvl[0]*lvl[2] - lvl[1]*lvl[3];
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
    int j=root_contour(inf), k=root_contour(sup);
    if(contours[j].lvl > contours[k].lvl)
        std::swap(j,k);
    Continuum c(j,k);
    c.mme.push_back(p);
    continua.push_back(c);
    return i;
}

/// Set contour at c2 have the same canonical element as the one at c1.
void CC::merge_contours(Pos c1, Pos c2) {
    assert(contours[idx(c1)].lvl == contours[idx(c2)].lvl);
    int i1 = root_contour(c1);
    int i2 = root_contour(c2);
    if(i1!=i2)
        contours[i2].parent = i1;
}

/// Find the canonical contour and perform path compression.
int CC::root_contour(int i) {
    int j=contours[i].parent;
    if(j<0)
        return i;
    return (contours[i].parent = root_contour(j));
}

/// Check whether dual pixel of top-left corner \a p is adjacent to edge of
/// top-left corner \a sep and orientation \a o (0=vertical, 1=horizontal).
int CC::adjacent_rect(const DPoint& p, Pos sep, int o) const {
    int oo=1-o;
    if((int)p[oo]==sep[oo] && (int)p[o]+1==sep[o] &&
       (p[o]!=(int)p[o] || contours[idx(Pos((int)p.x,(int)p.y+h))].p.x<0))
        return -1; // p above sep
    if((int)p[oo]==sep[oo] && (int)p[o]==sep[o] && p[o]==(int)p[o])
        return 1; // p below sep
    return 0;
}

/// When two continua meeting along edge of top-left \a sep have mme
/// \a v1 and \a v2, append v2 \a v1. They may have to be reordered so that
/// the edge is no longer a boundary. The orientation of the edge is given by
/// o (0=vertical, 1=horizontal).
/// Return iterator to the first element of junction.
std::vector<DPoint>::iterator
CC::merge_mme(std::vector<DPoint>& v1, std::vector<DPoint>& v2, Pos sep, int o) {
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

/// Find the canonical contour and perform path compression.
int CC::root_continuum(int i) {
    int j=continua[i].parent;
    if(j<0)
        return i;
    return (continua[i].parent = root_continuum(j));
}
