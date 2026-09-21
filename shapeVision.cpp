// SPDX-License-Identifier: MPL-2.0
/**
 * @file shapeVision.cpp
 * @brief shapeVision: Persistence map of bilinear image
 * @author Pascal Monasse <pascal.monasse@enpc.fr>
 * @date 2025-2026
 */

#include "cmdLine.h"
#include "io_png.h"
#include "cc.h"
#include "sample_ll.h"
#include "draw_curve.h"
#include <cmath>
#include <sstream>

struct Crop {
    int x, y, w, h;
    Crop(int w0=0, int h0=0): x(0), y(0), w(w0), h(h0) {}
};

std::ostream& operator<<(std::ostream& s, const Crop& C) {
    return s << C.w << 'x' << C.h << '+' << C.x << '+' << C.y;
}    

std::istream& operator>>(std::istream& str, Crop& C) {
    std::string s;
    str >> s;
    if(str.fail()) return str;
    std::istringstream is(s);
    is >> C.w;
    if(is.fail()) // w is optional
        is.clear();
    char c=0;
    is >> c;
    if(c!='x') {
        str.setstate(std::ios::failbit);
        return str;
    }
    is >> C.h;
    if(is.fail()) // h is optional
        is.clear();
    c=0;
    is >> c;
    if(c!='+') {
        if(! is.eof())
            str.setstate(std::ios::failbit);
        return str;
    }
    is >> C.x >> C.y;
    if(is.fail())
        str.setstate(std::ios::failbit);
    return str;
}

struct TransformZoom : public TransformPoint {
    int z, cx, cy;
    TransformZoom(int zoom, int x, int y): z(zoom), cx(x), cy(y) {}
    DPoint operator()(const DPoint& p) const {
        return DPoint(z*(p.x-cx), z*(p.y-cy));
    }
};

struct color_t {
    unsigned char r,g,b;
    color_t(): r(255), g(255), b(255) {}
    color_t(unsigned char r0, unsigned char g0, unsigned char b0)
    :r(r0),g(g0),b(b0) {}
};

bool output_persistence(const CC& cc,
                        const std::vector<Extremum>& ext,
                        const std::string& file,
                        const TransformPoint& t, int w, int h) {
    DPoint tl(0,0);
    tl = t(tl);
    DPoint br(w-1, h-1);
    br = t(br);
    w=std::ceil(br.x-tl.x)+1; h=std::ceil(br.y-tl.y)+1;
    color_t* im = new color_t[w*h];
    for(size_t i=0, n=ext.size(); i<n; i++) {
        float v = ext[i].plevel;
        std::vector<int>::const_iterator it, end=ext[i].boundaries.end();
        for(it=ext[i].boundaries.begin(); it!=end; ++it) {
            std::vector<DPoint> curve =
                sample_ll(cc.continua[*it], v, cc, 5);
            draw_curve(curve, color_t(255,0,0), im, w, h, t);
        }
    }
    bool ok=(0==io_png_write_u8(file.c_str(),(unsigned char*)im,w,h,3));
    delete [] im;
    return ok;
}

void display_stats(const CC& cc) {
    std::cout << "dual pixels: " << (cc.w-1)*(cc.h-1) << ", ";
    int n=0;
    for(int i=0; i<4; i++) {
        std::list<std::list<int>>::const_iterator it=cc.R.chainCode[i].begin(),
            end=cc.R.chainCode[i].end();
        for(; it!=end; ++it)
            n += it->size();
    }
    assert(n%2 == 0);
    n /= 2;
    std::cout << "continua: " << cc.continua.size() << " ("
              << n << " open, " << cc.continua.size()-n << " closed)\n"
              << cc.minima.size() << " minima, "
              << cc.maxima.size() << " maxima"
              << std::endl;
}

/** \mainpage ShapeVision.
  * Persistence maps of image obtained by bilinear interpolation of the samples.
*/
int main(int argc, char* argv[]) { 
    // parse arguments
    CmdLine cmd;
    int z=1;
    std::string min, max;
    Crop crop;
    cmd.add( make_option('m', min, "min")
             .doc("min-persistence output image") );
    cmd.add( make_option('M', max, "max")
             .doc("max-persistence output image") );
    cmd.add( make_option('z', z, "zoom")
             .doc("Zoom factor (integer) for output images") );
    cmd.add( make_option('c', crop, "crop")
             .doc("wxh+x+y = rect [x,x+w]x[y,y+h]") );
    try {
        cmd.process(argc, argv);
    } catch(const std::string& s) {
        std::cerr << "Error: " << s << std::endl;
        return 1;
    }
    if(argc != 2) {
        std::cerr << "Usage: " << argv[0] << " [options] imgIn.png\n"
                  << cmd;
        std::cerr << "Crop: w=0 or omitted means image right. "
                  << "h=0 means image height. "
                  << "+x+y is optional" << std::endl;
        return 1;
    }

    size_t w, h;
    float* im = io_png_read_f32_gray(argv[1], &w, &h);
    if(! im) {
        std::cerr << "Unable to load image " << argv[1] << std::endl;
        return 1;
    }
    if(crop.w==0)
        crop.w = w-crop.x;
    if(crop.h==0)
        crop.h = h-crop.y;
    if(crop.w<=0 || crop.h<=0) {
        std::cerr << "Crop region is outside image" << std::endl;
        return 1;
    }

    TransformZoom zoom(z, crop.x, crop.y);
    CC cc(im,(int)w,(int)h);
    display_stats(cc);
    if(! min.empty() &&
       !output_persistence(cc, cc.minima, min, zoom, crop.w, crop.h)) {
        std::cerr << "Error saving image file " << min << std::endl;
        return 1;
    }
    if(! max.empty() &&
       !output_persistence(cc, cc.maxima, max, zoom, crop.w, crop.h)) {
        std::cerr << "Error saving image file " << max << std::endl;
        return 1;
    }

    free(im);
    return 0;
}
