#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define SPACE_RANGE_X 16
#define SPACE_RANGE_Y 16
#define SPACE_RANGE_Z 16

#define PI 3.14159265
#define WIDTH 100
#define HEIGHT 100
#define RADIUS 10
//#define NOISE 0.1



pcl::PointCloud<pcl::PointXYZ> getOneSphere() {
    /* get center coordinate */
    //srand (time(NULL));
    float x = ((double) rand() / (RAND_MAX)) * SPACE_RANGE_X * 2 - SPACE_RANGE_X;
    float y = ((double) rand() / (RAND_MAX)) * SPACE_RANGE_Y * 2 - SPACE_RANGE_Y;
    float z = ((double) rand() / (RAND_MAX)) * SPACE_RANGE_Z * 2 - SPACE_RANGE_Z;
    //std::cout << "x: " << x << "y: " << y << "z: " << z << std::endl;
  
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width    = WIDTH;
    cloud.height   = HEIGHT;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);


    float angle_height = PI/cloud.height;
    float angle_width = 2*PI/cloud.width;

    //std::cout << angle_height << std::endl;
    //std::cout << angle_width << std::endl;

    for (size_t i = 0; i < cloud.width; i++) {
        for (size_t j = 0; j < cloud.height; j++) {
            cloud.points[i*cloud.height + j].x = sin(angle_height*j)*RADIUS*cos(angle_width*i) + x; 
            cloud.points[i*cloud.height + j].y = sin(angle_height*j)*RADIUS*sin(angle_width*i) + y;
            cloud.points[i*cloud.height + j].z = cos(angle_height*j)*RADIUS + z;
        }
    }

    return cloud;
}

void placeCube() {

}

pcl::PointCloud<pcl::PointXYZ> createObjects(std::string shape, int num) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 0;
    cloud.height = 0;
    cloud.is_dense = false;
    cloud.points.resize(0);
    
    srand (time(NULL));

    for (int i = 0; i < num; i++) {
        if (shape == "s") {
            cloud += getOneSphere();
        } else if (shape == "c") {
            placeCube();
        }
    }
    return cloud;
} 

   

int main (int argc, char** argv)
{
    if (argc != 3) {
        std::cout << "Usage: " << std::endl;
        std::cout << "pcloud_generator shape number" << std::endl;
        std::cout << "shape: s(sphere)/c(cube)" << std::endl;
        std::cout << "number: number of object" << std::endl;
        return (0);
    }

    std::string shape = argv[1];
    int num = atoi(argv[2]);

    std::cout << shape << std::endl;
    std::cout << num << std::endl;

    pcl::PointCloud<pcl::PointXYZ> cloud = createObjects(shape, num);

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

    //for (size_t i = 0; i < cloud.points.size (); ++i)
    //    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    return (0);
}
