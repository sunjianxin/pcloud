#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#define SPACE_RANGE_X   2
#define SPACE_RANGE_Y   2
#define SPACE_RANGE_Z   2
#define SCALE_MIN       1 //scale lower bound
#define SCALE_MAX       3 //scale upper bound

#define PI              3.14159265
#define WIDTH           100
#define HEIGHT          100
#define RADIUS          10
#define EDGE            100 //number of points on an edge of a cube
//#define NOISE 0.1

// shift PC's centroid to origin
void center(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    float x_sum = 0;
    float y_sum = 0;
    float z_sum = 0;
    for (int i = 0; i < cloud->points.size(); ++i){
        x_sum += cloud->points[i].x;
        y_sum += cloud->points[i].y;
        z_sum += cloud->points[i].z;
    }
    float x_centroid = x_sum/cloud->points.size();
    float y_centroid = y_sum/cloud->points.size();
    float z_centroid = z_sum/cloud->points.size();
    std::cout << "         centroid: x: " << x_centroid << " y: " << y_centroid << " z: " << z_centroid << std::endl;

    //shift point cloud to centroid
    for (int i = 0; i < cloud->points.size(); ++i){
        cloud->points[i].x = cloud->points[i].x - x_centroid;
        cloud->points[i].y = cloud->points[i].y - y_centroid;
        cloud->points[i].z = cloud->points[i].z - z_centroid;
    }
}

// shift PC to positive range of x, y, z axises
void addNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float nrr_x, float nrr_y, float nrr_z, float nnr) {
   
    // calculate the new cube size with noise
    float x_min, x_max, y_min, y_max, z_min, z_max;
    x_min = cloud->points[0].x;
    x_max = cloud->points[0].x;
    y_min = cloud->points[0].y;
    y_max = cloud->points[0].y;
    z_min = cloud->points[0].z;
    z_max = cloud->points[0].z;
    for (int i = 0; i < cloud->points.size(); ++i){
        if (cloud->points[i].x < x_min){
                x_min = cloud->points[i].x;
        }
        if (cloud->points[i].x > x_max){
                x_max = cloud->points[i].x;
        }
        if (cloud->points[i].y < y_min){
                y_min = cloud->points[i].y;
        }
        if (cloud->points[i].y > y_max){
                y_max = cloud->points[i].y;
        }
        if (cloud->points[i].z < z_min){
                z_min = cloud->points[i].z;
        }
        if (cloud->points[i].z > z_max){
                z_max = cloud->points[i].z;
        }
    }
    //std::cout << "x_min " << x_min << std::endl;
    //std::cout << "y_min " << y_min << std::endl;
    //std::cout << "z_min " << z_min << std::endl;

    float x_neg = x_min * nrr_x;
    float x_pos = x_max * nrr_x;
    float y_neg = y_min * nrr_y;
    float y_pos = y_max * nrr_y;
    float z_neg = z_min * nrr_z;
    float z_pos = z_max * nrr_z;
    //std::cout << "x_min " << x_neg << std::endl;
    //std::cout << "y_min " << y_neg << std::endl;
    //std::cout << "z_min " << z_neg << std::endl;
    //std::cout << "x_max " << x_pos << std::endl;
    //std::cout << "y_max " << y_pos << std::endl;
    //std::cout << "z_max " << z_pos << std::endl;




    // calculate the number of noise points
    int noise_num = (int)(cloud->points.size() * nnr);
    std::cout << "PC size: " << cloud->points.size() << std::endl;
    std::cout << "noice size: " << noise_num << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noise (new pcl::PointCloud<pcl::PointXYZ>);
    // Fill in the cloud data
    cloud_noise->width    = 1;
    cloud_noise->height   = noise_num;
    cloud_noise->is_dense = false;
    cloud_noise->points.resize(noise_num);

    for (int i = 0; i < noise_num; i++) {
        float x_noise = ((double) rand() / (RAND_MAX)) * (x_pos - x_neg) + x_neg;
        float y_noise = ((double) rand() / (RAND_MAX)) * (y_pos - y_neg) + y_neg;
        float z_noise = ((double) rand() / (RAND_MAX)) * (z_pos - z_neg) + z_neg;
        cloud_noise->points[i].x = x_noise;
        cloud_noise->points[i].y = y_noise;
        cloud_noise->points[i].z = z_noise;
        
    }

    *cloud += *cloud_noise;
    
}


void shiftToPositive(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    float x_min, x_max, y_min, y_max, z_min, z_max;
    x_min = cloud->points[0].x;
    x_max = cloud->points[0].x;
    y_min = cloud->points[0].y;
    y_max = cloud->points[0].y;
    z_min = cloud->points[0].z;
    z_max = cloud->points[0].z;
    for (int i = 0; i < cloud->points.size(); ++i){
        if (cloud->points[i].x < x_min){
                x_min = cloud->points[i].x;
        }
        if (cloud->points[i].x > x_max){
                x_max = cloud->points[i].x;
        }
        if (cloud->points[i].y < y_min){
                y_min = cloud->points[i].y;
        }
        if (cloud->points[i].y > y_max){
                y_max = cloud->points[i].y;
        }
        if (cloud->points[i].z < z_min){
                z_min = cloud->points[i].z;
        }
        if (cloud->points[i].z > z_max){
                z_max = cloud->points[i].z;
        }
    }
    for (int i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].x = cloud->points[i].x - x_min;
        cloud->points[i].y = cloud->points[i].y - y_min;
        cloud->points[i].z = cloud->points[i].z - z_min;
    }

}




int main (int argc, char** argv)
{
    if (argc < 6) {
        std::cout << "Usage: " << std::endl;
        std::cout << "pcloud_generator input_PC nrr_x nrr_y nrr_z nnr" << std::endl;
        std::cout << "input_PC: path to input pcd file" << std::endl;
        std::cout << "nrr_x/y/z(noise range ratio): range x, y, z dimension of the resulting file with noise, it defines size of the noise region that surrounds the original point cloud" << std::endl;
        std::cout << "nnr(noise number ratio): ratio of the number of noise point comparing to the point size of the original point cloud" << std::endl;
        return (0);
    }

    float nrr_x = atof(argv[2]);
    float nrr_y = atof(argv[3]);
    float nrr_z = atof(argv[4]);
    float nnr = atof(argv[5]);
    std::cout << "xx" << nrr_x << nrr_y << nrr_z << nnr << std::endl;

    // Load file | Works with PCD and PLY files
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile (argv[1], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[1] << std::endl << std::endl;
      return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = source_cloud;
    center(cloud);
    addNoise(cloud, nrr_x, nrr_y, nrr_z, nnr);
    shiftToPositive(cloud);

    pcl::io::savePCDFileASCII("result.pcd", *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to result.pcd." << std::endl;

    return (0);
}
