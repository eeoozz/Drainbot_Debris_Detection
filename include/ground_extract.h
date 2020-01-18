/**
 * To remove the points near the ground and extract the clusters of chockages from the rest of the points
 * The user can extract a slice of the point cloud at a self-defined distance along self-defined axis and project it into self-defined plane for shape analysis.
 The default setting is to project the cloud along z-axis into xy-plane.
 They can also project the whole point cloud into planes of uniform interval.
 */
#ifndef GROUND_EXTRACT_H
#define GROUND_EXTRACT_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>
#include "utils.h"

class GrdExt {
public:
  /**
 * A constructor.
 */
  GrdExt():cloud_aft_rm(new pcl::PointCloud<pcl::PointXYZ>),cloud(new pcl::PointCloud<pcl::PointXYZ>),cloud_ori(new pcl::PointCloud<pcl::PointXYZ>) {};
  /**
  * Structure containing the information of one chockage extracted from the point cloud after the removal of ground.
  */
  struct Obstacles {
    pcl::PointCloud<pcl::PointXYZ> cHull_points;/**< points at the border of the cluster of points for convex hull construction.*/
    std::vector< pcl::Vertices > polyg;/**< the resultant convex hull polygons, as a set of vertices.*/
    pcl::PointXYZ centroid;/**< coordinates of the centroid of the chockage.*/
    float volume;/**< estimated volume of the chockage in litre.*/
    float width,height,depth;/**< the width, height and depth of the chockage estimated from the convex hull constructed.*/
  };
  //std::vector<pcl::PointXYZ> visual_obs;
  /**
  * The point cloud after removal of points near the ground.
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aft_rm;
  /**
  * A list of chockages (if any).
  */
  std::vector<Obstacles> obstacles;
  /**
  * The matrix of control points for B-spline plane model.
  */
  std::vector<std::vector<pcl::PointXYZ> > ctrl_pts;
  /**
  * The cluster of point cloud as the candidate of the chockages.
  */
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds_result;
  /**
  * An indicator on whether there exists oversized chockages.
  */
  bool overSize = false;
  /** @brief To set the inputs to the ground removal process.
  @param cloud_ori Input point cloud.
  @param key_pts Input key_pts used as feature points to the B-spline function.
  @param transform The tranformation matrix. For correcting the coordinates calculation if the point cloud has been realigned with z-axis.
  */
  void setInput(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori,std::vector<pcl::PointXYZ>& key_pts,Eigen::Matrix4f& transform);
  /** @brief Calculate the describers for the B-spline model of the ground.
  @param degree Degree of the B-spline. Default is 1, which is in the form of straight lines.
  @param clamped Whether the the B-spline is clamped at the ends. Default is true.
  @param ctrl_num_z the number of the control number along z axis. Default is 10.
  */
  void getDescribers(int degree=1,bool clamped=true,int ctrl_num_z=10);
  /** @brief To remove the overlapped obstacles (which is unlikely to be real chockage) in the visual representation.
  @param overlappedIdx Index of the overlapped chockage that needed to be removed from the results. Gained from reconstruct part.
  */
  void fixObstacles(std::vector<int>& overlappedIdx);
  /** @brief To Calculate the chockages in the scene after the B-spline model is described.
  */
  void getObstacles() {
    GrdExt::groundRemove();
    GrdExt::calculateObstacles();
  }
private:
  //for describing the shape
  /**
  * Maximun and minimun value of the point cloud along the z axis (for B-spline model).
  */
  pcl::PointXYZ min_pt,max_pt;
  /**
  * Knots calculated along y-axis and z-axis (for B-spline model).
  */
  std::vector<int> knots_z,knots_y;
  /**
  * Degree, number of control points along y-axis and z-axis (for B-spline model).
  */
  int degree,ctrl_num_z,ctrl_num_y;
  /**
  * Sample interval along y-axis and z-axis (for B-spline model).
  */
  int samp_interval_y,samp_interval_z;
  /**
  * Whether the B-spline model is clamped.
  */
  bool clamped;
  /**
  * The tranformation matrix. For correcting the coordinates calculation if the point cloud has been realigned with z-axis.
  */
  Eigen::Matrix4f transform_matrix;

  /**
  * 2D key points from the shape fitting process.
  */
  std::vector<pcl::PointXYZ> key_pts;
  /**
  * Slightly modified version of the 2D key points to have a better removal of the points along the slope.
  */
  std::vector<pcl::PointXYZ> key_pts_filter;
  /**
  * Original input point cloud and the point cloud after downsampling.
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,cloud_ori;
  /**
  * Further clusters of points needed to be classified.
  */
  std::vector<pcl::PointCloud<pcl::PointXYZ>> amends_result;

  /** @brief To remove the points around the ground model in the point cloud.
  */
  void groundRemove();
  /** @brief To calculate the results chockages.
  */
  void calculateObstacles();
  /** @brief To get the convex hulll representation of the candidate clusters and classify it.
  @param cloud_in Input point cloud.
  @return Whether it is a chockage or not.
  */
  bool getVolPolyg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
  /** @brief Extract the clusters from the left points after removing the points around the ground model. Euclidean clustering is used here.
  @param toler Tolerance for euclidean clustering.
  @param min_cluster_size Minimum number of points for the cluters recognized in euclidean clustering.
  @param max_cluster_size Maximum number of points for the clusters recognized in eucledean clustering.
  */
  void getCloudsResult(float toler=0.045,int min_clus_size=30,int max_clus_size=20000);//toler=0.015
  //void getVisualObs();
  //when only use one row of key pts as a representation
  /** @brief To generate the B-spline surface model if only one 2D key points is given.
  */
  void geneFrSingleRow();
  /** @brief To calculate the B-spline control point.
  @param t Ty gained from the function getTy.
  @param i Iterator 1.
  @param k Iterator 2.
  @param knots Knots calculated.
  */
  float spline_bfunc(float t, int i, int k,std::vector<int>& knots);
  /** @brief To calculate the knots for B-splines.
  @param degree Degree of B-spline.
  @param ctl_size Number of control points.
  @param clamped Whether the B-spline is clamped or not.
  @return Calculated knots.
  */
  std::vector<int> create_knots(int degree, int ctl_size, bool clamped);
  /** @brief To calculate the sample interval for the B-splines.
  @param knots Knots of the B-spline.
  @param ctrl_pts_num Number of control points.
  @param degree Degree of the B-spline.
  @param clamped Whether it is clamped or not.
  @return sample interval calculated.
  */
  int getSampleInterval(std::vector<int>& knots,int ctrl_pts_num, int degree, bool clamped);
  /** @brief To calculate the position of the points within the range of point cloud along y-axis.
  @param pt Point that the range is required to calculated.
  @param key_pts Key points from the shape recognition.
  @return The value represents showing the position of the point along the y-axis.
  */
  float getTy(pcl::PointXYZ pt,std::vector<pcl::PointXYZ>& key_pts);
  /** @brief To check whether a cluster of points is on top of the ground model or not.
  @param centroid Centroid of the clusters.
  @param chull_pts Convex Hull points.
  @return whether the object is on the top of the ground or not.
  */
  bool objectOnTop(pcl::PointXYZ& centroid,pcl::PointCloud<pcl::PointXYZ>& chull_pts);
  /** @brief To calculate the centroid of one input point cloud (average).
  @param cloud The input point cloud.
  @return The centroid.
  */
  pcl::PointXYZ getCentroid(pcl::PointCloud<pcl::PointXYZ>& cloud);
  /** @brief To check whether a cluster of points is floating from the ground model.
  @param cloud_in The input point cloud.
  @return Whether the object is floating in the air or not.
  */
  bool objectFloat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
  /** @brief To calculate the length, width and height of the Convex Hull.
  @param temp_ob Candidate chockages.
  */
  void getLengthWidthHeight(Obstacles& temp_ob);
  /** @brief Initial checking process for the candidate chockages. Mainly to eniminate the false positive from the potential reflective objects in the water.
  @param cloud_in Candidate chockages.
  @return Whether the cluster has passed the initial check.
  */
  bool initialObjectCheck(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
};

#endif
