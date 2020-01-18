#include <utils.h>
#include <iostream>
#include <model_evaluate.h>

void ModelEvaluate::getMatchingAccuracy(std::vector<pcl::PointXYZ>& key_pts,matchingAccuracy& accuracy) {

  float left,right,dist;
  left = 0.5*(key_pts[0].x+key_pts[1].x);
  right = 0.5*(key_pts[4].x+key_pts[5].x);
  dist = left-right;
  std::cout << "*********dist:" << fabs(dist) << std::endl;
  std::cout << key_pts[4].x << " " << key_pts[5].x << " " <<right << std::endl;
  if (fabs(dist)>0.05) {
    accuracy.symmetry_check = 0;
    if ((left<-0.09)||(left>0.03))
      accuracy.symmetry_sect.push_back(0);
    if ((right<-0.09)||(right>0.03))
      accuracy.symmetry_sect.push_back(4);
  }
  else {
    accuracy.symmetry_check = 1;
    float diff = key_pts[0].x - key_pts[5].x;
    if (diff>0)
      key_pts[5].x += 0.9*diff;
    else
      key_pts[0].x -= 0.9*diff;
    diff = key_pts[1].x - key_pts[4].x;
    if (diff>0)
      key_pts[4].x += 0.9*diff;
    else
      key_pts[1].x -= 0.9*diff;
  }

    accuracy.length_check = 1;
    for (int i=0;i<key_pts.size()-1;i+=4) {
      if (pow(Utils::SqDist(key_pts[i],key_pts[i+1]),0.5)<0.2) {
        accuracy.length_check = 0;
        accuracy.length_sect.push_back(i);
      }
    }

  if ((fabs(key_pts[2].x-key_pts[3].x)>0.04)||(key_pts[2].x<-0.25)||(key_pts[3].x<-0.25))
    accuracy.bottom_check = 0;
  else
    accuracy.bottom_check = 1;


}

void ModelEvaluate::adjustKeyPts() {
  if (matchingAccurate(accuracy1)==0) {
    if ((accuracy1.bottom_check) && (accuracy2.bottom_check)) {
      if (fabs((key_pts1[2].x+key_pts1[3].x-key_pts2[2].x-key_pts2[3].x)*0.5)>0.05) {
        key_pts1[2].x = 0.2*key_pts1[2].x+0.8*key_pts2[2].x;
        key_pts1[3].x = 0.2*key_pts1[3].x+0.8*key_pts2[3].x;
      }
    }
    return;
  }
  else if ((matchingAccurate(accuracy2)==0) && (matchingAccurate(accuracy1)>1)) {
    key_pts1 = key_pts2;
    for (int i=0;i<key_pts1.size();i++)
      key_pts1[i].x += 0.01;
  }
  else {
    if (!accuracy1.symmetry_check) {
      if (accuracy1.symmetry_sect.size() == 1) {
        float diff = 0.5*(key_pts1[4].x+key_pts1[5].x) - 0.5*(key_pts1[0].x+key_pts1[1].x);
        if (accuracy1.symmetry_sect[0] == 0) {
          key_pts1[0].x += 0.8*diff;
          key_pts1[1].x += 0.8*diff;
        }
        else {
          key_pts1[4].x -= 0.8*diff;
          key_pts1[5].x -= 0.8*diff;
        }
      }
      else if (accuracy2.symmetry_check) {
        float diff = 0.02+0.25*(key_pts2[4].x+key_pts2[5].x+key_pts2[0].x+key_pts2[1].x-key_pts1[4].x-key_pts1[5].x-key_pts1[0].x-key_pts1[1].x);
        key_pts1[0].x += 0.8*diff;
        key_pts1[1].x += 0.8*diff;
        key_pts1[4].x += 0.8*diff;
        key_pts1[5].x += 0.8*diff;
      }
      else {
        key_pts1[0].x = key_pts1[5].x = -0.035;
        key_pts1[1].x = key_pts1[4].x = -0.06;
      }
    }

    if (!accuracy1.length_check) {
      if (accuracy1.length_sect.size() == 1) {
        if (accuracy1.length_sect[0] == 0) {
          //std::cout << "++++++++++++++++++++++++lengthcheck 0 " << std::endl;
          float dist = pow(Utils::SqDist(key_pts1[4],key_pts1[5]),0.5);
          key_pts1[1].getVector3fMap() = dist/pow(Utils::SqDist(key_pts1[0],key_pts1[1]),0.5)*\
          (key_pts1[1].getVector3fMap()- key_pts1[0].getVector3fMap()) + key_pts1[0].getVector3fMap();
          pcl::PointXYZ pt_balance = key_pts1[4];
          pt_balance.y = 0.15-pt_balance.y;
          key_pts1[1].getVector3fMap() = 0.5*(key_pts1[1].getVector3fMap()+pt_balance.getVector3fMap());
        }
        else {
          //std::cout << "++++++++++++++++++++++++lengthcheck 1 " << std::endl;
          float dist = pow(Utils::SqDist(key_pts1[0],key_pts1[1]),0.5);
          key_pts1[4].getVector3fMap() = dist/pow(Utils::SqDist(key_pts1[4],key_pts1[5]),0.5)*\
          (key_pts1[4].getVector3fMap()- key_pts1[5].getVector3fMap()) + key_pts1[5].getVector3fMap();
          pcl::PointXYZ pt_balance = key_pts1[1];
          pt_balance.y = 0.15-pt_balance.y;
          key_pts1[4].getVector3fMap() = 0.5*(key_pts1[4].getVector3fMap()+pt_balance.getVector3fMap());
        }
      }
      else if (accuracy2.length_check) {
        //std::cout << "++++++++++++++++++++++++lengthcheck accuracy2 " << std::endl;
          float dist = (pow(Utils::SqDist(key_pts2[0],key_pts2[1]),0.5)+pow(Utils::SqDist(key_pts2[4],key_pts2[5]),0.5))*0.5;
          key_pts1[1].getVector3fMap() = \
          dist/pow(Utils::SqDist(key_pts1[0],key_pts1[1]),0.5)*(key_pts1[1].getVector3fMap()- key_pts1[0].getVector3fMap()) + key_pts1[0].getVector3fMap();
          key_pts1[4].getVector3fMap() = \
          dist/pow(Utils::SqDist(key_pts1[4],key_pts1[5]),0.5)*(key_pts1[4].getVector3fMap()- key_pts1[5].getVector3fMap()) + key_pts1[5].getVector3fMap();
        }
      else {
        //std::cout << "++++++++++++++++++++++++lengthcheck else " << std::endl;
        key_pts1[1].getVector3fMap() = 2*(key_pts1[1].getVector3fMap()- key_pts1[0].getVector3fMap()) + key_pts1[0].getVector3fMap();
        key_pts1[4].getVector3fMap() = 2*(key_pts1[4].getVector3fMap()- key_pts1[5].getVector3fMap()) + key_pts1[5].getVector3fMap();
      }
    }

    if (!accuracy1.bottom_check) {
      int count = 0;
      for (int i=2;i<4;i++) {
        if ((key_pts1[i].x<-0.21)||(key_pts1[i].x>-0.17))
          count++;
      }
      if (count == 1) {
        //std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&count =1" << std::endl;
        for (int i=2;i<4;i++) {
          if ((key_pts1[i].x<-0.21)||(key_pts1[i].x>-0.17)) {
            key_pts1[i].getVector3fMap() = key_pts1[5-i].getVector3fMap();
            key_pts1[i].y = 0.15-key_pts1[i].y;
          }
        }
      }
      else if (accuracy2.bottom_check) {
        //std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&accuracy2.bottom_check"<<std::endl;
        for (int i=2;i<4;i++) {
          if ((key_pts1[i].x<-0.21)||(key_pts1[i].x>-0.17)) {
            key_pts1[i].x = key_pts2[i].x+0.01;
          }
        }
      }
      else {
          //std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&both"<<std::endl;
          float ratio = (-0.21-key_pts1[1].x)/(key_pts1[2].x-key_pts1[1].x);
          std::cout << ratio << std::endl;
            key_pts1[2].getVector3fMap() = ratio*(key_pts1[2].getVector3fMap()- key_pts1[1].getVector3fMap()) + key_pts1[1].getVector3fMap();
          ratio = (-0.21-key_pts1[4].x)/(key_pts1[3].x-key_pts1[4].x);
          std::cout << ratio << std::endl;
            key_pts1[3].getVector3fMap() = ratio*(key_pts1[3].getVector3fMap()- key_pts1[4].getVector3fMap()) + key_pts1[4].getVector3fMap();
      }
    }


  }
}

void ModelEvaluate::printAccuracy(matchingAccuracy& accuracy) {
  std::cout << "length check: " << accuracy.length_check << std::endl;//
  for (int i=0;i<accuracy.length_sect.size();i++)
    std::cout << accuracy.length_sect[i] << std::endl;
  std::cout << "symmetry check: " << accuracy.symmetry_check << std::endl;
  for (int i=0;i<accuracy.symmetry_sect.size();i++)
    std::cout << accuracy.symmetry_sect[i] << std::endl;
  std::cout << "bottom check: " << accuracy.bottom_check << std::endl;
}

int ModelEvaluate::matchingAccurate(matchingAccuracy& accuracy) {
  int temp = 0;
  if (accuracy.symmetry_check == 0)
    temp++;
  if (accuracy.length_check == 0)
    temp++;
  if (accuracy.bottom_check == 0)
    temp++;

  return temp;
}
