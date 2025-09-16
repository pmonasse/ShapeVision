#include "cc.h"
#include <string>
#include <iostream>

int main(int argc, char* argv[]) {
    size_t w=argc/2, h=2; 
    float* v = new float[w*h];
    for(int i=0; i+1<argc; i++)
        v[i] = std::stod(argv[i+1]);
    for(int i=argc; i<w*h; i++)
        v[i] = 0;
    std::cout << "Image dimension: " << w << 'x' << h << std::endl;
    for(int i=0,k=0; i<h; i++) {
        for(int j=0; j<w; j++, k++) {
            std::cout << v[k] << ' ';
        }
        std::cout << std::endl;
    }
    CC cc(v,w,h);
    return 0;
}
