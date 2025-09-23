#include "cc.h"
#include "cmdLine.h"
#include <string>
#include <iostream>

int main(int argc, char* argv[]) {
    size_t w=0, h;
    CmdLine cmd;
    cmd.add( make_option('w', w).doc("width of image (0=automatic)") );
    try {
        cmd.process(argc, argv);
    } catch(const std::string& s) {
        std::cerr << "Error: " << s << std::endl;
        return 1;
    }
    if(argc<2) {
        std::cerr << "Usage: " << argv[0] << "v1 [v2 [v3]...]" << std::endl;
        return 1;
    }
    if(w==0) {
        w=argc/2;
        h=2;
    } else {
        h=(argc-1+w-1)/w;
    }
    std::cout << "Image dimension: " << w << 'x' << h << std::endl;
        
    float* v = new float[w*h];
    for(int i=0; i+1<argc; i++)
        v[i] = std::stod(argv[i+1]);
    for(int i=argc; i<w*h; i++)
        v[i] = 0;
    for(int i=0,k=0; i<h; i++) {
        for(int j=0; j<w; j++, k++) {
            std::cout << v[k] << ' ';
        }
        std::cout << std::endl;
    }
    CC cc(v,w,h);
    return 0;
}
