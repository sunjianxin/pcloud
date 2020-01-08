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

### 2. add_noise
This is a program to add noise to a PC

    $ ./add_noise/build/add_noise [input PC path] [nrr_x] [nrr_y] [nrr_z] [nnr]

Where
* input PC path:
    * Path to the user defined PC for add noise
* nrr_x:
    * Noise range ratio on x dimension
* nnr_y:
    * Noise range ratio on y dimension
* nnr_z:
    * Noise range ratio on z dimension
* nnr:
    * Noise number ration, which is the ratio of the number of noise point comparing to the point size of the original PC



## Build

    $ cd (utilities folder)
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Prebuild Binary Tools
Useful prebuild binary tools are in "tools" folder which includes:

* ply2pcd:
For transform point cloud file from .ply to .pcd format
* pcl_viewer:
For visualizing .pcd files

## Dataset
Useful data generated by pcloud_generator is stored in "data" folder which includes:

* standard_cube.pcd: 
100x100x100 points cube with interval of 0.01, so the size of the cube is 0.99x0.99x0.99. It centroid is at origin.
* standard_cube_positive.pcd:
Same as standard_cube.pcd, but it is shift such that all 3 axisex starts from 0 to its positive ranges.
* bunny_pcd/bun_zipper_res3.pcd:
Standford bunny point cloud. 
