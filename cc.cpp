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
            const Contour& ctr = cc.contours[idx];
            for(int i=0; i<4; i++) {
                DPoint p = min(pos2DPoint(vo[i]),ctr.p);
                c[i] = cc.create_continuum(vo[i], s, p);
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

    DPoint dtl = pos2DPoint(tl);
    int eMin = edge_id(rank[0], rank[1]);
    chainCode[eMin].back().push_back(cc.idx(vo[0]));
    if(lvl[rank[0]]==lvl[rank[1]])
        cc.merge_contours(vo[0],vo[1]);
    else {
        c[0] = cc.create_continuum(vo[0],vo[1], dtl);
        chainCode[eMin].back().push_back(c[0]);
        chainCode[eMin].back().push_back(cc.idx(vo[1]));
    }

    int eMax = edge_id(rank[2], rank[3]);
    chainCode[eMax].back().push_back(cc.idx(vo[2]));
    if(lvl[rank[2]]==lvl[rank[3]])
        cc.merge_contours(vo[2],vo[3]);
    else {
        c[1] = cc.create_continuum(vo[2],vo[3], dtl);
        chainCode[eMax].back().push_back(c[1]);
        chainCode[eMax].back().push_back(cc.idx(vo[3]));
    }

    if((rank[1]+rank[2])&1) { // two adjacent intermediate level vertices
        int eInt = edge_id(rank[1],rank[2]); // intermediate edge
        chainCode[eInt].back().push_back(cc.idx(vo[1]));
        if(lvl[rank[1]]==lvl[rank[2]])
            cc.merge_contours(vo[1],vo[2]);
        else {
            c[2] = cc.create_continuum(vo[1],vo[2], dtl);
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
            c[2] = cc.create_continuum(vo[1],vo[2], dtl);
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

/// Find in chainCode \a L the continuum of index \a iSplit, that should be
/// present at most once. Insert before the continuum \a iCtn and the contour
/// \a iCtr. This is used during propagation of continua and contour when
/// splitting a continuum.
bool insert_chainCode(std::list<int>& L, int iSplit, int iCtn, int iCtr) {
    std::list<int>::iterator it = L.begin();
    for(++it; it!=L.end(); advance(it,2))
        if(*it == iSplit) {
            L.insert(it, iCtn);
            L.insert(it, iCtr);
            return true;
        }
    return false;
}

/// The continuum of index \a iSplit must be split by the continuum \a iCtn
/// with delimiting contour \a iCtr. This adds to the chain-code at the exit
/// (last MME) these references. 
void split_continuum(CC& cc, Rect& R, int iSplit, int iCtn, int iCtr) {
    cc.continua[iSplit].infCtr = iCtr;
    const DPoint& p = cc.continua[iSplit].mme.back();
    std::list<int>::iterator it;
    if(p.y == R.tl.y) { // Upper edge
        std::list<std::list<int>>::iterator i = R.chainCode[0].begin();
        std::advance(i, (int)p.x-R.tl.x);
        if( insert_chainCode(*i, iSplit, iCtn, iCtr) )
            return;
    }
    if(p.x == R.tl.x) { // Left edge
        std::list<std::list<int>>::iterator i = R.chainCode[3].begin();
        std::advance(i, (int)p.y-R.tl.y);
        if( insert_chainCode(*i, iSplit, iCtn, iCtr) )
            return;
    }
    DPoint q = cc.mme_br(p);
    if(p.x == R.br.x) { // Right edge
        std::list<std::list<int>>::iterator i = R.chainCode[1].begin();
        std::advance(i, (int)p.y-R.tl.y);
        if( insert_chainCode(*i, iSplit, iCtn, iCtr) )
            return;
    }
    if(q.y == R.br.y) { // Bottom edge
        std::list<std::list<int>>::iterator i = R.chainCode[2].begin();
        std::advance(i, (int)p.x-R.tl.x);
        if( insert_chainCode(*i, iSplit, iCtn, iCtr) )
            return;
    }
    assert(false);
}

/// Given two chain-codes \a L1 and \a L2 along a common edge, merge or split
/// continua, merge contours. The vertical common edge has top at \a sep. The
/// enclosing rectangles \a R1 and \a R2 are horizontally adjacent.
void propagate(CC& cc, Rect& R1, Rect& R2, Pos sep,
               const std::list<int>& L1, const std::list<int>& L2) {
    assert(!L1.empty() && !L2.empty());
    assert(L1.back()==L2.back());
    std::list<int>::const_iterator i1=L1.begin(), i2=L2.begin();
    assert(*i1 == *i2);
    ++i1; ++i2;
    if(i1 == L1.end()) { // Single contour
        assert(i2 == L2.end());
        return;
    }
    int ic1 = cc.root_continuum(*i1++);
    int ic2 = cc.root_continuum(*i2++);
    int j1 = cc.root_contour(*i1++);
    int j2 = cc.root_contour(*i2++);
    int l1 = cc.contours[j1].lvl;
    int l2 = cc.contours[j2].lvl;
    do {
        if(l1 == l2) {
            if(j1 != j2)
                cc.contours[j2].parent = j1;
            if(ic1 != ic2) {
                cc.merge_mme(cc.continua[ic1].mme, cc.continua[ic2].mme, sep);
                cc.continua[ic2].parent = ic1;
                cc.continua[ic2].mme.clear();
                cc.continua[ic2].mme.shrink_to_fit();
            }
        } else if(l1<l2) { // split continuum ic2
            cc.merge_mme(cc.continua[ic1].mme, cc.continua[ic2].mme, sep);
            split_continuum(cc, R2, ic2, ic1, j1);
        } else if(l2<l1) { // split continuum ic1
            cc.merge_mme(cc.continua[ic2].mme, cc.continua[ic1].mme, sep);
            split_continuum(cc, R1, ic1, ic2, j2);
        }
        int l1old=l1;
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
    } while(i1!=L1.end() && i2!=L2.end());
}

/// Merge two adjacent rectangles, separated by vertical edges.
Rect merge_horizontal(CC& cc, Rect& R1, Rect& R2) {
    assert(Pos(R1.br.x,R1.tl.y) == R2.tl); // tr(R1)=tl(R2)
    assert(R1.br == Pos(R2.tl.x,R2.br.y)); // br(R1)=bl(R2)

    // Propagate chain-codes along common edges
    assert(R1.chainCode[1].size()==R2.chainCode[3].size());
    std::list<std::list<int>>::const_iterator i1=R1.chainCode[1].begin(),
                                              i2=R2.chainCode[3].begin(),
                                              end=R1.chainCode[1].end();
    Pos sep = R2.tl;
    for(; i1!=end; ++i1, ++i2, ++sep.y)
        propagate(cc, R1, R2, sep, *i1, *i2);

    // Move chain-codes at frame of R
    Rect R(R1.tl, R2.br);
    std::swap(R1.chainCode[0],R.chainCode[0]);
    R.chainCode[0].splice(R.chainCode[0].end(),R2.chainCode[0]);
    std::swap(R1.chainCode[2],R.chainCode[2]);
    R.chainCode[2].splice(R.chainCode[2].end(),R2.chainCode[2]);
    std::swap(R1.chainCode[3],R.chainCode[3]);
    std::swap(R2.chainCode[1],R.chainCode[1]);
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
        for(int i=0; i<h2; i++)
            for(int j=0; j+1<w2; j+=2) {
                Rect r = merge_horizontal(*this, R[i*h2+j], R[i*h2+j+1]);
                R.push_back(r);
            }
        R.erase(R.begin(), R.begin()+n);
        w2=(w2+1)/2;
        // Vertical propagation: TODO
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

/// When two continua meeting along vertical edge of top \a sep have mme
/// \a v1 and \a v2, append v2 \a v1. They may have to be reordered so that
/// the edge is no longer a boundary.
void CC::merge_mme(std::vector<DPoint>& v1, std::vector<DPoint>& v2, Pos sep) {
    const DPoint& p = v1.front();
    if((int)p.y==sep.y && (int)p.x+1==sep.x &&
       (p.x!=(int)p.x || contours[idx(Pos((int)p.x,(int)p.y+h))].p.x<0)) 
        reverse(v1.begin(), v1.end());
    const DPoint& q = v2.back();
    if((int)q.y==sep.y && (int)q.x==sep.x && q.x==(int)q.x)
        reverse(v2.begin(), v2.end());
    v1.insert(v1.end(), v2.begin(), v2.end());
}

/// Find the canonical contour and perform path compression.
int CC::root_continuum(int i) {
    int j=continua[i].parent;
    if(j<0)
        return i;
    return (continua[j].parent = root_continuum(j));
}
