/**
 * To get 2D slice of the point cloud reconstructed
 * The user can extract a slice of the point cloud at a self-defined distance along self-defined axis and project it into self-defined plane for shape analysis.
 The default setting is to project the cloud along z-axis into xy-plane.
 They can also project the whole point cloud into planes of uniform interval.
 */
#ifndef CLOUD_SLICING_H
#define CLOUD_SLICING_H

#include <pcl/ModelCoefficients.h>

class CloudSlice {
public:
  /**
 * A constructor.
 */
  CloudSlice():coefficient(new pcl::ModelCoefficients),cloud_ori(new pcl::PointCloud<pcl::PointXYZ>),sect_range(0.008),sect_dist(0.02) {
    coefficient->values.resize (4);
    coefficient->values = {0,0,1,0}; /**< plane descriper. the projection plane is the xy plane */
  };
  /** @brief Set the input point cloud for slicing
  @param name The file name of the point cloud (pcd format under the directory)
  @param downsample For the large-sized point cloud to be downsampled with voxel grid size of 8mm. The default is false.
 */
  void setInputCloud(std::string name,bool downsample=false);
  /** @brief Another way to set the input point cloud
  @param this_cloud The input point cloud
  @param downsample For the large-sized point cloud to be downsampled with voxel grid size of 0.008m. The default is false.
 */
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr this_cloud,bool downsample=false);
  /** @brief Optional function to customize the section range
  @param sect_range The section range chosen by the user. Default value is 0.008m
 */
  void setSectRange(float sect_range);
  /** @brief Optional function to customize the section interval along the z axis
  @param sect_dist The interval for section chosen by the user. Default value is 2cm
 */
  void setSectDist(float sect_dist);
  //optional, default:0,0,1,0
  /** @brief Optional function to customize the plane for projection. The default one is {0,0,1,0}, which represents the xy-plane
  @param a Coefficient value 1
  @param b Coefficient value 2
  @param c Coefficient value 3
  @param d Coefficient value 4
 */
  void setPlaneIdx (float& a,float& b, float& c,float& d) {
    coefficient->values = {a,b,c,d};
  }
  /** @brief Get the 2D slice at the distance defined along the z-axis
  @param axis_z the position along the z-axis
 */
  void sectProject(float axis_z);
  /**
  *The original input cloud
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori;
  /**
  *The result extracted output cloud.
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extract;
  /**
  *maximum and minimum point along the z-axis.
  */
  pcl::PointXYZ min_pt, max_pt;
  /**
  *Transform matrix to align the point cloud along z-axis.
  */
  Eigen::Matrix4f transform_matrix;

private:
  float sect_range,sect_dist;
  /**
  *Number of sections projected, depends on the section distance defined.
  */
  int sect_num;//number of sections based on the sect dist set before
  //plane coefficient
  /**
  *Model coefficients for the plane projected to.
  */
  pcl::ModelCoefficients::Ptr coefficient;
  /**
  Get the number of sections if want to get the point cloud sliced in regular interval.
  */
  void getSectNum();
  /**
  *Helper function for sect_project.
  */
  void getOneSect(float axis_z);
  /**
  *Extract the points within the area of interest from the original input point cloud_in.
  */
  void limitViewRange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

};
#endif
