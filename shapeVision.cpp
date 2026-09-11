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

struct TransformZoom : public TransformPoint {
    int z;
    TransformZoom(int zoom=1): z(zoom) {}
    DPoint operator()(const DPoint& p) const {
        return DPoint(z*p.x, z*p.y);
    }
};

struct color_t {
    unsigned char r,g,b;
    color_t(): r(255), g(255), b(255) {}
    color_t(unsigned char r0, unsigned char g0, unsigned char b0)
    :r(r0),g(g0),b(b0) {}
};

bool output_persistence(const CC& cc,
                        const std::vector<int>& ext,
                        const std::vector<float>& lvl,
                        const std::vector<std::vector<int>>& boundaries,
                        const std::string& file, const TransformPoint& t) {
    DPoint tl(0,0);
    tl = t(tl);
    DPoint br(cc.w-1, cc.h-1);
    br = t(br);
    int w=std::ceil(br.x-tl.x)+1, h=std::ceil(br.y-tl.y)+1;
    color_t* im = new color_t[w*h];
    for(size_t i=0, n=ext.size(); i<n; i++) {
        float v = lvl[i];
        std::vector<int>::const_iterator it, end=boundaries[i].end();
        for(it=boundaries[i].begin(); it!=end; ++it) {
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
    for(int i=0, end=2*cc.w*cc.h; i<end; i++)
        if(cc.contours[i].parent<0 && cc.contours[i].p.x>=0)
            ++n;
    std::cout << "contours: " << n << ", ";
    n = 0;
    for(int i=0; i<4; i++) {
        std::list<std::list<int>>::const_iterator it=cc.R.chainCode[i].begin(),
            end=cc.R.chainCode[i].end();
        for(; it!=end; ++it)
            n += it->size();
    }
    assert(n%2 == 0);
    n /= 2;
    std::cout << "continua: " << cc.continua.size() << " ("
              << n << " open, " << cc.continua.size()-n << " closed)"
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
    cmd.add( make_option('m', min, "min")
             .doc("min-persistence output image") );
    cmd.add( make_option('M', max, "max")
             .doc("max-persistence output image") );
    cmd.add( make_option('z', z, "zoom")
             .doc("Zoom factor (integer) for output images") );
    try {
        cmd.process(argc, argv);
    } catch(const std::string& s) {
        std::cerr << "Error: " << s << std::endl;
        return 1;
    }
    if(argc != 2) {
        std::cerr << "Usage: " << argv[0] << " [options] imgIn.png\n"
                  << cmd;
        return 1;
    }

    size_t w, h;
    float* im = io_png_read_f32_gray(argv[1], &w, &h);
    if(! im) {
        std::cerr << "Unable to load image " << argv[1] << std::endl;
        return 1;
    }

    TransformZoom zoom(z);
    CC cc(im,(int)w,(int)h);
    display_stats(cc);
    if(! min.empty() &&
       !output_persistence(cc, cc.minima, cc.persistLevelMin, cc.boundariesMin,
                           min, zoom)) {
        std::cerr << "Error saving image file " << min << std::endl;
        return 1;
    }
    if(! max.empty() &&
       !output_persistence(cc, cc.maxima, cc.persistLevelMax, cc.boundariesMax,
                           max, zoom)) {
        std::cerr << "Error saving image file " << max << std::endl;
        return 1;
    }

    free(im);
    return 0;
}
