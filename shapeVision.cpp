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

/// Region of original image to crop
struct Crop {
    int x, y, w, h;
    Crop(int w0=0, int h0=0): x(0), y(0), w(w0), h(h0) {}
};

/// Output crop region
std::ostream& operator<<(std::ostream& s, const Crop& C) {
    return s << C.w << 'x' << C.h << '+' << C.x << '+' << C.y;
}

/// Input crop region.
/// Format is wxh+x+y or wxh. Value 0 for w or h means to image right or bottom.
/// \a w can be omitted (equivalent 0). If omitted, +x+y means +0+0.
std::istream& operator>>(std::istream& str, Crop& C) {
    C = Crop();
    std::string s;
    str >> s;
    if(str.fail()) return str;
    std::istringstream is(s);
    is >> C.w;
    if(is.fail()) // w is optional
        is.clear();
    char c=0;
    is >> c >> C.h;
    if(c!='x' || is.fail()) {
        str.setstate(std::ios::failbit);
        return str;
    }
    if(is.eof()) // +x+y is optional
        return str;
    c=0;
    is >> c >> C.x >> C.y;
    if(c!='+' || is.fail())
        str.setstate(std::ios::failbit);
    return str;
}

/// Zoom from given center
struct TransformZoom : public TransformPoint {
    int z, cx, cy;
    TransformZoom(int zoom, int x, int y): z(zoom), cx(x), cy(y) {}
    DPoint operator()(const DPoint& p) const {
        return DPoint(z*(p.x-cx), z*(p.y-cy));
    }
};

/// Pixel color
struct color_t {
    unsigned char r,g,b;
    color_t(): r(255), g(255), b(255) {}
    color_t(unsigned char r0, unsigned char g0, unsigned char b0)
    :r(r0),g(g0),b(b0) {}
};

/// Save persistence boundary image.
/// \a s is the sampling step, points can be transformed by zoom \a t.
/// \a w and \a h determine the dimensions of original image that are drawn.
bool output_persistence(const CC& cc,
                        const std::vector<Extremum>& ext,
                        const std::string& file, int s,
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
                sample_ll(cc.continua[*it], v, cc, s);
            draw_curve(curve, color_t(255,0,0), im, w, h, t);
        }
    }
    bool ok=(0==io_png_write_u8(file.c_str(),(unsigned char*)im,w,h,3));
    delete [] im;
    return ok;
}

/// Info about contours and continua
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
    int z=1, s=1;
    std::string min, max;
    Crop crop;
    cmd.add( make_option('m', min, "min")
             .doc("min-persistence output image") );
    cmd.add( make_option('M', max, "max")
             .doc("max-persistence output image") );
    cmd.add( make_option('z', z, "zoom")
             .doc("Zoom factor (positive integer) for output images") );
    cmd.add( make_option('c', crop, "crop")
             .doc("wxh+x+y = rect [x,x+w]x[y,y+h]") );
    cmd.add( make_option('s', s, "sampling")
             .doc("samples (>=0) per pixel unit") );
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
    if(z<1) {
        std::cerr << "Zoom factor z must be positive" << std::endl;
        return 1;
    }
    if(s<0) {
        std::cerr << "Sampling step s must be non-negative" << std::endl;
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
       !output_persistence(cc, cc.minima, min, s, zoom, crop.w, crop.h)) {
        std::cerr << "Error saving image file " << min << std::endl;
        return 1;
    }
    if(! max.empty() &&
       !output_persistence(cc, cc.maxima, max, s, zoom, crop.w, crop.h)) {
        std::cerr << "Error saving image file " << max << std::endl;
        return 1;
    }

    free(im);
    return 0;
}
