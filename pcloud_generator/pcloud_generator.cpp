#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#define SPACE_RANGE_X 0.25
#define SPACE_RANGE_Y 0.25
#define SPACE_RANGE_Z 0.25
#define SCALE_MIN 1 //scale lower bound
#define SCALE_MAX 3 //scale upper bound

#define PI 3.14159265
#define WIDTH 100
#define HEIGHT 100
#define RADIUS 10
//#define NOISE 0.1

// show pc info
void centerPC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    float temp_length;
    //get z_maz: tree height
    float tree_height;
    float x_min, x_max, y_min, y_max, z_min, z_max;
    float x_sum = 0;
    float y_sum = 0;
    float z_sum = 0;
    x_min = cloud->points[0].x;
    x_max = cloud->points[0].x;
    y_min = cloud->points[0].y;
    y_max = cloud->points[0].y;
    z_min = cloud->points[0].z;
    z_max = cloud->points[0].z;
    tree_height = 0;
    for (int i = 0; i < cloud->points.size(); ++i){
        x_sum += cloud->points[i].x;
        y_sum += cloud->points[i].y;
        z_sum += cloud->points[i].z;
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
    float x_centroid = x_sum/cloud->points.size();
    float y_centroid = y_sum/cloud->points.size();
    float z_centroid = z_sum/cloud->points.size();
    tree_height = z_max - z_min;
    std::cout << "Input PC height: " << tree_height << std::endl;
    std::cout << "         x range: " << x_min << " ~ " << x_max << std::endl; 
    std::cout << "         y range: " << y_min << " ~ " << y_max << std::endl; 
    std::cout << "         z range: " << z_min << " ~ " << z_max << std::endl;
    std::cout << "         centroid: x: " << x_centroid << " y: " << y_centroid << " z: " << z_centroid << std::endl;

    //shift point cloud to centroid
    for (int i = 0; i < cloud->points.size(); ++i){
        cloud->points[i].x = cloud->points[i].x - x_centroid;
        cloud->points[i].y = cloud->points[i].y - y_centroid;
        cloud->points[i].z = cloud->points[i].z - z_centroid;
    }

}


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


pcl::PointCloud<pcl::PointXYZ>::Ptr getRandomTransformPC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    /* fill a randome rotation parameters */
    float x_angle = ((double) rand() / (RAND_MAX)) * M_PI;
    float y_angle = ((double) rand() / (RAND_MAX)) * M_PI;
    float z_angle = ((double) rand() / (RAND_MAX)) * M_PI;
    Eigen::Matrix4f x_rotation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f y_rotation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f z_rotation = Eigen::Matrix4f::Identity();
    z_rotation (0,0) = std::cos(z_angle);
    z_rotation (0,1) = -sin(z_angle);
    z_rotation (1,0) = sin(z_angle);
    z_rotation (1,1) = std::cos(z_angle);
    y_rotation (0,0) = std:: cos(y_angle);
    y_rotation (0,2) = -sin(y_angle);
    y_rotation (2,0) = sin(y_angle);
    y_rotation (2,2) = std::cos(y_angle);
    x_rotation (1,1) = std::cos(x_angle);
    x_rotation (1,2) = -sin(x_angle);
    x_rotation (2,1) = sin(x_angle);
    x_rotation (2,2) = std::cos(x_angle);
    Eigen::Matrix4f rotation = x_rotation*y_rotation*z_rotation;
    //std::cout << "Rotation M: " << std::endl;
    //std::cout << rotation << std::endl;
    
    /* fill a random scaling parameters */
    Eigen::Matrix4f scaling = Eigen::Matrix4f::Identity();
    float scale = ((double) rand() / (RAND_MAX)) * (SCALE_MAX - SCALE_MIN) + SCALE_MIN;
    scaling(0,0) = scale;
    scaling(1,1) = scale;
    scaling(2,2) = scale;
    //std::cout << "Scaling M: " << std::endl;
    //std::cout << scaling << std::endl;

    /* fill a random translation parameters */
    Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
    float x = ((double) rand() / (RAND_MAX)) * SPACE_RANGE_X * 2 - SPACE_RANGE_X;
    float y = ((double) rand() / (RAND_MAX)) * SPACE_RANGE_Y * 2 - SPACE_RANGE_Y;
    float z = ((double) rand() / (RAND_MAX)) * SPACE_RANGE_Z * 2 - SPACE_RANGE_Z;
    translation(0, 3) = x;
    translation(1, 3) = y;
    translation(2, 3) = z;       /* Do translation */
    //std::cout << "Translation M: " << std::endl;
    //std::cout << translation << std::endl;

    /* aggregated transformation matrix */
    Eigen::Matrix4f transform = rotation*scaling*translation;

    /* do transformation */
    pcl::transformPointCloud(*cloud, *transform_cloud, transform);
    return transform_cloud;
}

   
pcl::PointCloud<pcl::PointXYZ> createObjectsUserDefine(std::string path, int num) {
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_user_define (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    reader.read(path, *cloud_user_define);

    centerPC(cloud_user_define);
    
    srand (time(NULL));

    for (int i = 0; i < num; i++) {
        *cloud += *getRandomTransformPC(cloud_user_define);
    }
    return *cloud;
}
   












int main (int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << std::endl;
        std::cout << "pcloud_generator shape number" << std::endl;
        std::cout << "shape: s(sphere)/c(cube)/u(userdefined)" << std::endl;
        std::cout << "number: number of object" << std::endl;
        return (0);
    }

    std::string shape = argv[1];
    int num = atoi(argv[2]);
    std::string path;
    if (argc == 4)
        path = argv[3];

    std::cout << shape << std::endl;
    std::cout << num << std::endl;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (shape == "u") {
        cloud = createObjectsUserDefine(path, num);
    } else {
        cloud = createObjects(shape, num);
    }

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

    //for (size_t i = 0; i < cloud.points.size (); ++i)
    //    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    return (0);
}
