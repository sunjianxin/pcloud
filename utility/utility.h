#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

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

// show pc info
void centerAndNormalizePC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
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

    //normalize point cloud
    float x_diff = x_max - x_min;
    float y_diff = y_max - y_min;
    float z_diff = z_max - z_min;
    std::cout << "input dimension: x " << x_diff << "; y " << y_diff << "; z " << z_diff << ";" << std::endl;
    //find the maxima
    float max = x_diff;
    if (max < y_diff)
        max = y_diff;
    if (max < z_diff)
        max = z_diff;
    std::cout << "maxima: " << max << std::endl;
    float ratio = 1/max;
    for (int i = 0; i < cloud->points.size(); ++i){
        cloud->points[i].x = cloud->points[i].x * ratio;
        cloud->points[i].y = cloud->points[i].y * ratio;
        cloud->points[i].z = cloud->points[i].z * ratio;
    }
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





// add noise to point cloud
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


