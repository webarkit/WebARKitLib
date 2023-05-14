# WebARKitLib

The C/C++ source code of WebARKit, from Artoolkit5 and extended.
Updated to [Eigen](https://eigen.tuxfamily.org) 3.4.0 and Emscripten emsdk 3.1.26.

## Features

The library containes the following features:
**Artoolkit5** C/C++ code used by [jsartoolkitNFT](https://github.com/webarkit/jsartoolkitNFT) and [ARnft](https://github.com/webarkit/ARnft) (see lib and include folders). Note that is not the full Artoolkit5 library, but only the code needed by jsartoolkitNFT and ARnft. The code is updated to Eigen 3.4.0 and can be compiled with Emscripten emsdk 3.1.26.
**WebARKit** C++ code, which is our experimental code for WebAR. It is completetly independent from Artoolkit5 and can be used as a standalone library. It can be compiled with Emscripten emsdk 3.1.26.
