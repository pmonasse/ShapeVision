// SPDX-License-Identifier: MPL-2.0
/**
 * @file shapeVision.cpp
 * @brief shapeVision: Persistence map of bilinear image
 * @author Pascal Monasse <pascal.monasse@enpc.fr>
 * @date 2025
 */

#include "cmdLine.h"
#include "io_png.h"
#include "cc.h"
using namespace std;

/** \mainpage ShapeVision.
  * Persistence maps of image obtained by bilinear interpolation of the samples.
*/
int main(int argc, char* argv[]) { 
    // parse arguments
    CmdLine cmd;

    /*    int nScales=0;
    float grad=0;
    
    // options
    cmd.add( make_option('s', nScales, "scales")
             .doc("nb scales (0=automatic)") );
    cmd.add( make_option('g', grad, "gradient")
             .doc("Min gradient norm (0=automatic)") );
    */
    try {
        cmd.process(argc, argv);
    } catch(const std::string& s) {
        std::cerr << "Error: " << s << std::endl;
        return 1;
    }
    if(argc != 2) {
        cerr << "Usage: " << argv[0] << " [options] imgIn.png\n"
             << cmd;
        return 1;
    }

    size_t w, h;
    float* im = io_png_read_f32_gray(argv[1], &w, &h);
    if(! im) {
        cerr << "Unable to load image " << argv[1] << endl;
        return 1;
    }

    CC cc(im,(int)w,(int)h); // Will do someghing with it later...

    free(im);
    return 0;
}
