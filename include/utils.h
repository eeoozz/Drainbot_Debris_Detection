/**
 * This class contains some universal utility functions which are used in other classes.
 */
#ifndef UTILS_H
#define UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

class Utils {
public:
  /** @brief Realign the point cloud along the z-axis with PCA calculation.
  @param cloud_in Input point cloud.
  @param cloud_out Output point cloud.
  @return Transformation matrix in Eigen matrix format.
  */
  static Eigen::Matrix4f cloudPCATransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
  /** @brief Downsample the point cloud with the voxel size defined.
  @param cloud_in Input point cloud.
  @param cloud_out Output point cloud.
  @param voxel_size User-defined voxel size. Default is 0.008 m.
  */
  static void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,\
                          float voxel_size=0.008);
  /** @brief To remove NAN value from the point cloud.
  @param cloud_in Input point cloud.
  @param cloud_out Output point cloud.
  */
  static void rmNAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
  /** @brief To project the point cloud to 2D plane
  @param cloud_in Input point cloud.
  @param cloud_out Output point cloud.
  @param coefficient Coefficients of the plane projected to.
  */
  static void cloud2PlaneProject(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,\
                                 pcl::ModelCoefficients::Ptr coefficient);
  /** @brief To project the point on the line defined.
  @param pt_in Input point.
  @param pt_out Output point.
  @param coefficient Coefficients of the line projected to.
  */
  static void pt2LineProject(pcl::PointXYZ& pt_in,pcl::PointXYZ& pt_out,pcl::ModelCoefficients& coefficient);
  /** @brief Another way to project the point on the line defined.
  @param pt_in Input point.
  @param pt_out Output point.
  @param pt_begin One point on the line projected.
  @param pt_end Another point on the line projected to.
  */
  static void pt2LineProject(pcl::PointXYZ& pt_in,pcl::PointXYZ& pt_out,pcl::PointXYZ& pt_begin,pcl::PointXYZ& pt_end);
  /** @brief To check if the number is one member of the indices.
  @num The number to be checked.
  @idx The indices list.
  */
  static bool isInVector(int num,std::vector<int> idx);
  /** @brief To calculate the square distance between two points.
  @param pt1 The first point.
  @param pt2 The second point.
  */
  static float SqDist(pcl::PointXYZ& pt1,pcl::PointXYZ& pt2);
  /** @brief To erase the noise from the point cloud. The distance of noise is defined by user through the value of clus_toler.
  @param cloud_in Input cloud.
  @param cloud_out Output cloud.
  @param clus_toler Cluster tolerance for noise removal.
  */
  static void eraseNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,float clus_toler);
  /** @brief To check if two line segment intersect or not.
  @param p1 Starting point on line segment 1.
  @param q1 Ending point on line segment 1.
  @param p2 Starting point on line segment 2.
  @param q2 Ending point on line segment 2.
  @return Whether two line segments intersect.
  */
  static bool doIntersect(pcl::PointXYZ& p1, pcl::PointXYZ& q1, pcl::PointXYZ& p2, pcl::PointXYZ& q2);
  /** @brief To check whether a point is within a polygon or not.
  @param polygon A series of points that defines the polygen.
  @param n Number of indices in the polygon.
  @param p Input point required to be checked.
  @return Whether the point is within polygon.
  */
  static bool isInside(pcl::PointCloud<pcl::PointXYZ> polygon, int n, pcl::PointXYZ p);
  /** @brief To apply the transformation matrix on a single point.
  @param transform Transformation matrix.
  @param point_in Input point.
  @param point_out Output point.
  */
  static void pointTransform(Eigen::Matrix4f& transform, pcl::PointXYZ& point_in,pcl::PointXYZ& point_out);
  /** @brief For sorting function to sort the point from small to large along y-axis.
  */
  static bool smallerPointY (pcl::PointXYZ& i,pcl::PointXYZ& j) { return (i.y<j.y); }
  /** @brief For sorting function to sort the point from large to small along y-axis.
  */
  static bool largerPointY (pcl::PointXYZ& i,pcl::PointXYZ& j) { return (i.y>j.y); }
  /** @brief For sorting function to sort the point from small to large along x-axis.
  */
  static bool smallerPointX (pcl::PointXYZ& i,pcl::PointXYZ& j) { return (i.x<j.x); }
  /** @brief For sorting function to sort the point from large to small along x-axis.
  */
  static bool largerPointX (pcl::PointXYZ& i,pcl::PointXYZ& j) { return (i.x>j.x); }
  /** @brief For sorting function to sort the point from small to large along z-axis.
  */
  static bool smallerPointZ (pcl::PointXYZ& i,pcl::PointXYZ& j) { return (i.z<j.z); }
private:
  static bool onSegment(pcl::PointXYZ& p, pcl::PointXYZ& q, pcl::PointXYZ& r);
  static int orientation(pcl::PointXYZ& p, pcl::PointXYZ& q, pcl::PointXYZ& r);
};
#endif
