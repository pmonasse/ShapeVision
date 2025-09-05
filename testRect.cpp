#include "cc.h"
#include <string>

int main(int argc, char* argv[]) {
    float v[4]={0,1,2,3};
    for(int i=0; i<4 && i+1<argc; i++)
        v[i] = std::stod(argv[i+1]);
    CC cc(v,2,2);
    return 0;
}
