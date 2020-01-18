/**
* To combine all the steps in the chockage detection
* Input will be stereo images in the drainage
* Results will be generated on whether there is chockage and whether it is oversized. The chockage will be marked in the image with a green rectangle
*/
#ifndef GENERATE_REPORT_H
#define GENERATE_REPORT_H

#include "opencv2/highgui.hpp"
#include <pcl/point_types.h>
#include <iostream>

class GeneReport {
public:
  /**
 * A constructor.
 @param left_image Left image from the stereo images in OpenCV mat format
 @param right_image Right image from the stereo images in OpenCV mat format
 */
  GeneReport(cv::Mat& left_image, cv::Mat& right_image) {
    this->left_image = left_image;
    this->right_image = right_image;
  };
  /**
 * A constructor.
 @param left_name file of the left image from the stereo images
 @param right_name file of the right_image from the stereo images
 */
  GeneReport(std::string left_name,std::string right_name) {
       left_image = cv::imread(left_name, 0);
       right_image = cv::imread(right_name,0);
  };
  /**
  * Output report show whether chockages exist and the details of each, also indicates that whether the chockage is oversized or not.
  */
  struct Report {
    bool blockExist; /**< indicator of whether the chockage exists or not. */
    bool overSizeExist; /**< indicator of whether the oversized chockage exists or not.*/
    int blockNum; /**< number of chockages (if no chockage exists, this value would be 0) */
    std::vector<pcl::PointXYZ> blockCoordinates; /**< the coordinates of each chockage (if any) from the left camera in metre.*/
    std::vector<float> blockSize; /**< the estimated volume of each chockage (if any) in litre.*/
  };
  /**
  * Left image from the stereo images.
  */
  cv::Mat left_image;
  /**
  * Right image from the stereo images.
  */
  cv::Mat right_image;
  /**
  * Left image with detected chockages marked in green rectangles.
  */
  cv::Mat image_marked;
  /**
  * Object of the report.
  */
  Report block_report;
  /**
  * Function called to do chockage analysis after set the input left image and right image.
  */
  void generateReport();
};

#endif
