/**
 * To evaluate the fitted 2D model and adjust the fitting accordingly.
 * Two series of key points from two random sections in the point cloud are evaluated and the final key points are calculated.
 * Based on some assumptions of the shape of the ground. Symmetry, length of each segment and lowest points are checked.
 */
#ifndef MODEL_EVALUATE_H
#define MODEL_EVALUATE_H

#include <iostream>
#include <string>

class ModelEvaluate {
public:
  /**
  * A constructor.
  @param key_pts1 The first series of key points from one random section of 2D projection within the point cloud.
  @param key_pts2 The second series of key points from another random section of 2D projection within the point cloud.
  */
  ModelEvaluate(std::vector<pcl::PointXYZ>& key_pts1, std::vector<pcl::PointXYZ>& key_pts2) {
    this->key_pts1 = key_pts1;
    this->key_pts2 = key_pts2;
  };
  /**
  * Structure containing the model evaluation results for one series of point cloud.
  */
  struct matchingAccuracy {
    bool length_check;/**< whether the length test (length of each line segment should not be too small) has passed. */
    std::vector<int> length_sect;/**< the index of the line segment that failed to pass the length test. */
    bool symmetry_check;/**< whether the symmetry test (whether the shape is near symmetrical) has passed. */
    std::vector<int> symmetry_sect;/**< the index of the line segment that failed to pass the symmtry test. */
    bool bottom_check;/**< whether the bottom test (if the bottom is too deep it indicates there might be reflection problem) has passed.*/
  };
  /**
  * Model evaluation results for key_pts1 and key_pts2 respectively.
  */
  matchingAccuracy accuracy1,accuracy2;
  /**
  * Two series of input key points from random section of 2D projection within the point cloud.
  */
  std::vector<pcl::PointXYZ> key_pts1,key_pts2;
  /** @brief To get the model evaluation results for one series of key points.
  @param key_pts Input key points.
  @param accuracy Output evaluation results.
  */
  void getMatchingAccuracy(std::vector<pcl::PointXYZ>& key_pts,matchingAccuracy& accuracy);
  /** @brief To adjust the final key points of the ground in the scene based on the two evaluation results.
  */
  void adjustKeyPts();
  /** @brief To print the model evaluation results.
  @warning Just for visualization and debugging purpose.
  */
  void printAccuracy(matchingAccuracy& accuracy);
private:
  int matchingAccurate(matchingAccuracy& accuracy);

};

#endif
