/*
 * @Description: ICP 匹配模块
 * @Author: Su
 * @Date: 2021-12-25
 */

// 构造最近临匹配方法，求两个点云之间点的对应关系
// 求点云平均值，点云的质心
// 使用IPC-SVD求解R和t
// 最难的是第一步，

#include "lidar_localization/models/registration/my_icp.hpp"
#include <pcl/common/transforms.h>

#include "glog/logging.h"

using namespace Eigen;

namespace lidar_localization {

MY_ICPRegistration::MY_ICPRegistration(const YAML::Node& node)
{
    
    max_corr_dist_ = node["max_corr_dist"].as<float>();
    trans_eps_ = node["trans_eps"].as<float>();
    euc_fitness_eps_ = node["euc_fitness_eps"].as<float>();
    max_iter_ = node["max_iter"].as<int>();

    last_pose = Eigen::Matrix4f::Identity();

}

MY_ICPRegistration::MY_ICPRegistration(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) {

    max_corr_dist_ = max_corr_dist;
    trans_eps_ = trans_eps;
    euc_fitness_eps_ = euc_fitness_eps;
    max_iter_ = max_iter;
}


bool MY_ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    
    // if(kdtree)
    //     delete kdtree;
    //pcl::search::KdTree<pcl::PointXYZ>::Ptr kd(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree.reset(new pcl::search::KdTree<pcl::PointXYZ>());

    kdtree -> setInputCloud(input_target);

    last_cloud = input_target;
    // computecentroid(last_cloud, last_centroid);


    return true;
}

bool MY_ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    
    result_pose = predict_pose;
    std::vector<int> indices_temp;
    int now_num = 0;
    float eps = euc_fitness_eps_ + 1;

    std::cout << "--------------" << std::endl;

    std::cout << "predict_pose:" << std::endl << predict_pose << std::endl;
    // std::cout << "last_pose: " << std::endl << last_pose << std::endl;

    for(; now_num < max_iter_ && eps >= euc_fitness_eps_ ; now_num ++)
    {
       
        // std::cout << result_pose << std::endl;

        pcl::transformPointCloud(*input_source, *result_cloud_ptr, result_pose);
        
        indices.clear();
        indices_temp.clear();
        eps = computemapping(result_cloud_ptr, indices_temp);
        // computecentroid(result_cloud_ptr, now_centroid);
        // computecentroid(last_cloud, last_centroid);

        // std::cout << "eps = " << eps << std::endl;

        compute_pose(result_cloud_ptr, result_pose);

        std::cout << "now_num: " << now_num << "\t" << "result_pose: " << std::endl << result_pose << std::endl;

    }
    // std::cout << result_pose << std::endl;

    // result_pose = last_pose * result_pose;

    // std::cout << "result_pose: " << std::endl << result_pose << std::endl << std::endl;
    
    last_pose = result_pose;

    return true;
}

void MY_ICPRegistration::compute_pose(CloudData::CLOUD_PTR& result_cloud_ptr, Eigen::Matrix4f& result_pose)
{
    Eigen::Matrix3f W = Eigen::Matrix3f::Zero();

    for(int i = 0; i < result_cloud_ptr->points.size(); i ++)
    {
        if(indices[i] == -1)
            continue;
        CloudData::POINT p = result_cloud_ptr->points[i];
        CloudData::POINT q = last_cloud -> points[indices[i]];

        W += Vector3f(p.x - now_centroid.x, p.y - now_centroid.y, p.z - now_centroid.z) * Vector3f(q.x - last_centroid.x, q.y - last_centroid.y, q.z - last_centroid.z).transpose();

    }
    Eigen::JacobiSVD<Matrix3f> svd(W, ComputeFullU | ComputeFullV);
    MatrixXf V = svd.matrixV();
    MatrixXf U = svd.matrixU();
    double determinant = W.determinant();
    Matrix3f R_ans = Matrix3f::Zero();

    if(determinant < 0)
    {
        R_ans = -V * U.transpose();
    }
    else
        R_ans = V * U.transpose();

    MatrixXf t_ans = Vector3f(last_centroid.x, last_centroid.y, last_centroid.z) - R_ans * Vector3f(now_centroid.x, now_centroid.y, now_centroid.z);

    Eigen::Matrix4f WW = Eigen::Matrix4f::Identity();

    for(int i = 0; i < 3; i ++)
        for(int j = 0; j < 3; j ++)
        {
            WW(i, j) = R_ans(i, j);
        }
    WW(0, 3) = t_ans(0, 0);
    WW(1, 3) = t_ans(1, 0);
    WW(2, 3) = t_ans(2, 0);

    result_pose = WW * result_pose ;
    
}

float MY_ICPRegistration::computemapping(CloudData::CLOUD_PTR& input_source, std::vector<int> indices_temp)
{
    std::vector<float> distances;
    float sum = 0;
    mapnum = 0;

    std::cout << "compute mapping start!" << std::endl;

    int i = 0;
    last_centroid.x = 0;
    last_centroid.y = 0;
    last_centroid.z = 0;
    now_centroid.x = 0;
    now_centroid.y = 0;
    now_centroid.z = 0;

    for(i = 0; i < input_source->points.size(); i ++)
    {
        pcl::PointXYZ point = input_source->points[i];
        kdtree->nearestKSearch(point, 1, indices_temp, distances);
        
        // std::cout << "distances[0] = " << distances[0] << std::endl;

        if(distances[0] < max_corr_dist_)
        {
            mapnum ++;
            indices.push_back(indices_temp[0]);
            sum += distances[0];

            last_centroid.x = last_centroid.x + last_cloud->points[indices_temp[0]].x;
            last_centroid.y = last_centroid.y + last_cloud->points[indices_temp[0]].y;
            last_centroid.z = last_centroid.z + last_cloud->points[indices_temp[0]].z;
            now_centroid.x = now_centroid.x + input_source->points[i].x;
            now_centroid.y = now_centroid.y + input_source->points[i].y;
            now_centroid.z = now_centroid.z + input_source->points[i].z;

        }
        else
        {
            indices.push_back(-1);
        }

        indices_temp.clear();
        distances.clear();
    }
    std::cout << "i = " << i << std::endl;
    std::cout << "num = " << mapnum << std::endl;
    std::cout << "sum = " << sum / mapnum << std::endl;
    last_centroid.x = last_centroid.x / mapnum;
    last_centroid.y = last_centroid.y / mapnum;
    last_centroid.z = last_centroid.z / mapnum;

    now_centroid.x = now_centroid.x / mapnum;
    now_centroid.y = now_centroid.y / mapnum;
    now_centroid.z = now_centroid.z / mapnum;

    return sum / mapnum;
    
}


void MY_ICPRegistration::computecentroid(CloudData::CLOUD_PTR& cloud, CloudData::POINT& point)
{
    point.x = 0;
    point.y = 0;
    point.z = 0;
    for(auto p : cloud->points)
    {
        point.x += p.x;
        point.y += p.y;
        point.z += p.z;
    }
    point.x /= cloud->points.size();
    point.y /= cloud->points.size();
    point.z /= cloud->points.size();
}

void MY_ICPRegistration::computecentroidcoordinate(CloudData::POINT& cloud, CloudData::POINT& centroid)
{

    cloud.x -= centroid.x;
    cloud.y -= centroid.y;
    cloud.z -= centroid.z;

}




// flloat MY_ICPRegistration::Distance(pcl::PointXYZ a, pcl::PointXYZ b)
// {
//     return square((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
// }

}