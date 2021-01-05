# Mandoline



### Description
Mandoline is 2D polygon slicing software. It performs a scanline segmentation of the polygon and connects the generated vertices along the edges of the polygon in a staggered manner to visit the entire surface of the polygon.

<<<<<<< HEAD
=======
Video of the provided example:

[![alt](https://img.youtube.com/vi/o7m5Jv45MrU/0.jpg)](https://www.youtube.com/watch?v=o7m5Jv45MrU "Click to play on YouTube")

>>>>>>> 852ca8a3d89f18267859ca1be967e5ca0833c2c8

### Building
Built on Ubuntu 20.04.1 LTS compiled with GCC 9.3.0.

Dependencies:
```
sudo apt install libeigen3-dev libcairomm-1.0-dev
```

To build & run the provided example:
```
mkdir build && cd build
cmake .. && make
./Mandoline
```
