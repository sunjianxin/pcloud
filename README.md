# pcloud
This is a research project for handling point cloud
## Utilities
### 1. pcloud_generator
This is a program to generate a PC with multiple object instance. Usage:

    $ ./pcloud_generator/build/pcloud [options] [input PC path]

Where

* options: 
    * s: Spheres
    * u: User defined
* input PC path:
    * Path to the user defined PC, only needed whtn option is u

## Build

    $ cd (utilities folder)
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
