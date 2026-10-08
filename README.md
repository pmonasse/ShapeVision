# ShapeVision
Copyright Martin Brooks, Pascal Monasse

## License
Mozilla Public License v2

## Build
```shell
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Run
```
Usage: ./build/shapeVision [options] imgIn.png
-m, --min=ARG min-persistence output image
-M, --max=ARG max-persistence output image
-z, --zoom=ARG Zoom factor (positive integer) for output images (1)
-c, --crop=ARG wxh+x+y = rect [x,x+w]x[y,y+h] (0x0+0+0)
-s, --sampling=ARG samples (>=0) per pixel unit (1)
-w, --min-weight=ARG Min number of extrema in persistence region (1)
Crop: w=0 or omitted means image right. h=0 means image height. +x+y is optional
'''

## Example
```
$ ./build/shapeVision z 100 -m small_min.png -M small_max.png data/small.png
dual pixels: 16, continua: 27 (19 open, 8 closed)
5 minima, 6 maxima
'''

```
$ ./build/shapeVision -w 50 -m enpc_min.png -M enpc_max.png data/enpc.png
dual pixels: 754171, continua: 431463 (15992 open, 415471 closed)
287 minima, 295 maxima
'''

Compare output images to those in data/
