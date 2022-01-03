/*
 * @Description: ICP 匹配模块
 * @Author: Su
 * @Date: 2021-12-25
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_MY_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_MY_ICP_REGISTRATION_HPP_

#include "lidar_localization/models/registration/registration_interface.hpp"
#include <pcl/search/kdtree.h>
#include <vector>

namespace lidar_localization {

class MY_ICPRegistration: public RegistrationInterface {
  public:
    MY_ICPRegistration(const YAML::Node& node);
    MY_ICPRegistration(
      float max_corr_dist, 
      float trans_eps, 
      float euc_fitness_eps, 
      int max_iter
    );

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
  
  private:
      float max_corr_dist_; 
      float trans_eps_;
      float euc_fitness_eps_; 
      int max_iter_;
      
      int mapnum;
      std::vector<int> indices;
      CloudData::POINT last_centroid;
      CloudData::POINT now_centroid;
      CloudData::CLOUD_PTR last_cloud;
      
      Eigen::Matrix4f last_pose;

      pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
      
      void computecentroid(CloudData::CLOUD_PTR& cloud, CloudData::POINT& point);
      void computecentroidcoordinate(CloudData::POINT& point, CloudData::POINT& centroid);
      void compute_pose(CloudData::CLOUD_PTR& result_cloud_ptr, Eigen::Matrix4f& result_pose);
      float computemapping(CloudData::CLOUD_PTR& input_source, std::vector<int> indices_temp);


//   private:
//    pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_;
};
}

#endif