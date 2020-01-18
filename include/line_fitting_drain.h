/**
 * To gain the shape of the ground through sequential line fitting
 * User could use functions in this class to find the line segment from a 2D point cloud. The number of line segment and the rangle of angle for each segment should be predefined.
   Or the search will be conducted 360 times for each segment.
 */
#ifndef LINE_FITTING_DRAIN_H
#define LINE_FITTING_DRAIN_H

#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/visualization/pcl_visualizer.h>

class LineRecogDrain {
public:
  /**
  * Constructor of the class.
  */
  LineRecogDrain():
    cloud (new pcl::PointCloud<pcl::PointXYZ>),
    cloud_ori (new pcl::PointCloud<pcl::PointXYZ>) {};
  /**
  * A structure storing definitions of the line segments recognized.
  */
  struct lineSeg {
    pcl::ModelCoefficients coefficientsSeg; /**< coefficients of the line segment. */
    pcl::PointXYZ ptBeg;/**< beginning point of the line segment. */
    pcl::PointXYZ ptEnd;/**< ending point of the line segment. */
    pcl::PointXYZ ptBeg_elong;/**< elonged point of the beginning point for calculating the intersecting key points. */
    pcl::PointXYZ ptEnd_elong;/**< elonged point of the ending point for calculating the intersecting key points. */
  };
  /**
  * Predefined range for each line segment to conduct searching in the size of x*2. x represents the number of segments.
  * e.g., line segment no.1 is at the rang from 20 degree to 90 degree. Then the first row in the matrix is [20 90].
  */
  std::vector<std::vector<int>> shape_definer;
  /**
  * A container for a series of line segments.
  */
  std::vector<lineSeg> lines;
  /**
  * Key points calculated to define the shape.
  */
  std::vector<pcl::PointXYZ> key_pts;
  /**
  * The original input point cloud.
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori;
  /**
  * The point cloud after noise removal and downsampling.
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  /**
  * Series of output searching rectangle for line segment. Only for visualization purpose.
  */
  std::vector<std::vector<pcl::PointXYZ>> rects;
  /** @brief To set input cloud for the class.
  @param name File name of the input cloud.
  */
  void setInputCloud(std::string name);
  /** @brief Another choice to set the input cloud for the class.
  @param this_cloud The input cloud of the class.
  */
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr this_cloud);
  /** @brief To calculate the line segments and the key points for the cloud.
  */
  void getSegsAndKeyPts();
  /** @brief To get the initial point for sequential line fitting.
  */
  void getInitialPoint();
  /** @brief to get the next line segment and next key point in the sequential line fitting.
  @param point_pre Previous point. Starting point of the line segment.
  @param lb Low bound for searching.
  @param hb Hight bound for searching.
  @param num_pt Number of point in the searching order.
  */
  void getFollowingPoint(pcl::PointXYZ& point_pre,int lb,int hb,int num_pt);

private:
  float y_smaller,y_bigger;
  pcl::PointXYZ end_point;

  void cloudTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  pcl::PointCloud<pcl::PointXYZ> getRect(pcl::PointXYZ& pt_beg, pcl::PointXYZ& pt_end, float width);
  pcl::PointXYZ getTestEndBound(pcl::PointXYZ& point_pre,int deg);
  bool readShapeDefiners();
  int findSlopeWithinRange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ& point_pre,int lb,int hb);
  void sortAlongOritation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,int max_deg);
  void getCurrentCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,pcl::PointXYZ point_pre);
  pcl::PointXYZ getSynmatricPt(pcl::PointXYZ& pt_in,pcl::PointXYZ& pt1, pcl::PointXYZ& pt2);
  void getPtElonged(pcl::PointXYZ& pt_beg,pcl::PointXYZ& pt_end,pcl::PointXYZ& pt_out);
};

#endif
