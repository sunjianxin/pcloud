#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include "../utility/utility.h"

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

int main (int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << std::endl;
        std::cout << "icp_test target_PC template_PC" << std::endl;
        std::cout << "target_PC: point cloud with multiple object instances" << std::endl;
        std::cout << "template_PC: point cloud of single template object" << std::endl;
        return (0);
    }
    
    // Load file | Works with PCD and PLY files
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_pc (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile (argv[1], *target_pc) < 0)  {
      std::cout << "Error loading point cloud " << argv[1] << std::endl << std::endl;
      return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_pc (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile (argv[2], *template_pc) < 0)  {
      std::cout << "Error loading point cloud " << argv[2] << std::endl << std::endl;
      return -1;
    }

    
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr transform_pc_test (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transform_test = Eigen::Matrix4f::Identity();
    transform_test(0, 0) = 2;
    transform_test(1, 1) = 2;
    transform_test(2, 2) = 2;
    transform_test(0,3) = 1;
    pcl::transformPointCloud(*template_pc, *target_pc, transform_test);



    //center(target_pc);
    //shiftToPositive(target_pc);
    //center(template_pc);
    //shiftToPositive(template_pc);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(template_pc);
    icp.setInputTarget(target_pc);
    icp.setMaximumIterations (100);
    //icp.setTransformationEpsilon (1e-14);
    //icp.setEuclideanFitnessEpsilon(1e-14);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(*Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    std::cout << icp.getEuclideanFitnessEpsilon() << std::endl;



    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_pc (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transform = icp.getFinalTransformation();
    pcl::transformPointCloud(*template_pc, *transform_pc, transform);

    //visulization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (target_pc, 255, 0, 0); // Red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (template_pc, 0, 255, 0); // Green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (transform_pc, 0, 0, 255); // Blue
    viewer->addPointCloud (target_pc, red, "target");
    viewer->addPointCloud (template_pc, green, "template");
    //viewer->addPointCloud (transform_pc, blue, "transform");
    viewer->addPointCloud (Final, blue, "transform");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "template");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "transform");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


    return (0);
}
