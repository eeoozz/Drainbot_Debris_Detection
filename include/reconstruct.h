/**
 * To reconstruct the point cloud of the scene from two stereo images captured.
 * This class also contains some functions to project the chockages recognized back to 2D images and highlight them with green rectangles.
 */
#ifndef RECONSTRUCT_H
#define RECONSTRUCT_H

#include <iostream>
#include <string>

#include "opencv2/highgui.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Reconstruct {
public:
  /**
  * A constructor.
  @param left_name File name of the left image in the stereo images.
  @param right_name File name of the right image in the stereo images.
  */
  Reconstruct(std::string left_name,std::string right_name):
    cloud (new pcl::PointCloud<pcl::PointXYZ>),cloud_vis (new pcl::PointCloud<pcl::PointXYZ>) {
       left_img = cv::imread(left_name, 0);
       right_img = cv::imread(right_name,0);
  };
  /**
  * A constructor.
  @param left Left image in the stereo images.
  @param right Right image in the stereo images.
  */
  Reconstruct(cv::Mat& left,cv::Mat& right):
    cloud (new pcl::PointCloud<pcl::PointXYZ>),cloud_vis (new pcl::PointCloud<pcl::PointXYZ>) {
        left_img = left;
        right_img = right;
  };

  /**
  * The calculated point cloud from the disparity map.
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  /**
  * The transformed point cloud used in the final visualization process.
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vis;
  /**
  * Left image from the stereo images.
  */
  cv::Mat left_img;
  /**
  * Right image from the stereo images.
  */
  cv::Mat right_img;
  /**
  * Left image after rectified.
  */
  cv::Mat left_rect;
  /**
  * Right image after rectified.
  */
  cv::Mat right_rect;
  /**
  * Disparity map computed.
  */
  cv::Mat disp_map;
  /**
  * Left image in color format to mark the chockage in green rectangle. The original one is grey image.
  */
  cv::Mat left_img_color;
  /**
  * List of the coordinates of the chockages detected in point cloud.
  */
  std::vector<pcl::PointXYZ> obs;
  /**
  * List of the coordinates of the chockage detected in 2D image.
  */
  std::vector<cv::Point> obs_img;
  /**
  * The indices of the potential overlapped chockages detected. The one further is probably a false positive and requires to be removed.
  */
  std::vector<int> overlappedIdx;
  /**
  * Matrix of undistorted rectified map.
  */
  cv::Mat rmap[2][2];

  /**@brief To compute the disparity map from the stereo images.
  @param no_downscale Whether the images are downscaled for computation. Default is true.
  */
  void getDisparityMap(bool no_downscale = true);
  /** @brief To calculate the point cloud.
  */
  void getPointCloud();
  /** @brief Set the input for chockages visualization on the original image.
  @param obs The coodinates of chockages.
  */
  void setVisualInput (std::vector<pcl::PointXYZ>& obs) {
    this->obs = obs;
  }
  /** @brief To get the chockages marked on the original images.
  */
  void getObjectsVisual();
private:
  cv::Mat matrix_Q;
  void getRectifiedImage();
  void rmOverlapped(std::vector<cv::Point>& marker, std::vector<int>& overlappedIdx);
};


#endif
