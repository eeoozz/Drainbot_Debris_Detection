#include <reconstruct.h>
#include <cloud_slicing.h>
#include <ground_extract.h>
#include <line_fitting_drain.h>
#include <model_evaluate.h>
#include <utils.h>
#include <generate_report.h>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

void GeneReport::generateReport() {
  Reconstruct reconst(left_image,right_image);
  reconst.getDisparityMap(true);
  reconst.getPointCloud();

  CloudSlice slice;
  slice.setInputCloud(reconst.cloud);
  slice.setSectRange(0.5);//0.3
  slice.sectProject(1.8);

  LineRecogDrain linerec;
  linerec.setInputCloud(slice.cloud_extract);
  linerec.getSegsAndKeyPts();


  //***********************************************
  CloudSlice slice2;
  slice2.setInputCloud(reconst.cloud);
  slice2.setSectRange(0.3);//0.3
  slice2.sectProject(1.2);

  LineRecogDrain linerec2;
  linerec2.setInputCloud(slice2.cloud_extract);
  linerec2.getSegsAndKeyPts();

  ModelEvaluate evaluation(linerec.key_pts,linerec2.key_pts);
  evaluation.getMatchingAccuracy(evaluation.key_pts1,evaluation.accuracy1);
  evaluation.getMatchingAccuracy(evaluation.key_pts2,evaluation.accuracy2);
  std::cout << "here ++" << std::endl;
  evaluation.adjustKeyPts();
  std::cout << "here" << std::endl;

  //***********************************************
  GrdExt rmGround;
  rmGround.setInput(slice.cloud_ori,evaluation.key_pts1,slice.transform_matrix);
  rmGround.getDescribers();
  rmGround.getObstacles();

  std::vector<pcl::PointXYZ> centroids;
  if (rmGround.obstacles.size() >0) {
    for (int i=0;i<rmGround.obstacles.size();i++)
      centroids.push_back(rmGround.obstacles[i].centroid);
    reconst.setVisualInput(centroids);
    reconst.getObjectsVisual();
  }
  if (reconst.overlappedIdx.size()>0)
    rmGround.fixObstacles(reconst.overlappedIdx);
  if (rmGround.obstacles.size() >0) {
    block_report.blockExist = true;
    block_report.overSizeExist = rmGround.overSize;
    block_report.blockNum = rmGround.obstacles.size();
    for (int i=0;i<rmGround.obstacles.size();i++) {
      block_report.blockCoordinates.push_back(rmGround.obstacles[i].centroid);
      block_report.blockSize.push_back(rmGround.obstacles[i].volume);
    }
  }
  else {
    block_report.blockExist = false;
    block_report.overSizeExist = false;
    block_report.blockNum = 0;
    block_report.blockCoordinates.clear();
    block_report.blockSize.clear();
  }
  image_marked = reconst.left_img_color;
}
