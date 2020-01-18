#include "reconstruct.h"
#include "utils.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

#include <pcl/io/pcd_io.h>

void Reconstruct::getDisparityMap(bool no_downscale) {
  Reconstruct::getRectifiedImage();
  cv::cvtColor(left_img, left_img_color, cv::COLOR_GRAY2BGR);

  int wsize;
  wsize = 3;//3

  int max_disp = 208;//208
  double lambda = 1000;//2000
  double sigma  = 1.5;//1.5
  double vis_mult = 1;//1
  //bool no_downscale = true;//false

  //cv::Mat left_rect  = cv::imread("rect_image_left.png" ,cv::IMREAD_COLOR);
  //cv::Mat right_rect  = cv::imread("rect_image_right.png" ,cv::IMREAD_COLOR);

  cv::Mat left_for_matcher, right_for_matcher;
  cv::Mat left_disp,right_disp;
  cv::Mat filtered_disp;
  cv::Mat conf_map = cv::Mat(left_rect.rows,right_rect.cols,CV_8U);
  conf_map = cv::Scalar(255);
  cv::Rect ROI;
  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
  double matching_time, filtering_time;

  if(!no_downscale)
  {
    // downscale the views to speed-up the matching stage, as we will need to compute both left
    // and right disparity maps for confidence map computation
    //! [downscale]
    max_disp/=2;
    if(max_disp%16!=0)
    max_disp += 16-(max_disp%16);
    resize(left_rect ,left_for_matcher ,cv::Size(),0.5,0.5, cv::INTER_LINEAR);
    resize(right_rect,right_for_matcher,cv::Size(),0.5,0.5, cv::INTER_LINEAR);
    //! [downscale]
  }
  else
  {
    left_for_matcher  = left_rect.clone();
    right_for_matcher = right_rect.clone();
  }

  //sgbm starts
  cv::Ptr<cv::StereoSGBM> left_matcher  = cv::StereoSGBM::create(0,max_disp,wsize);
  left_matcher->setP1(24*wsize*wsize);//24
  left_matcher->setP2(1000*wsize*wsize);//96
  left_matcher->setPreFilterCap(63);//63
  left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
  wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
  cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

  left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
  right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);


  //! [filtering]
  wls_filter->setLambda(lambda);
  wls_filter->setSigmaColor(sigma);
  wls_filter->filter(left_disp,left_rect,filtered_disp,right_disp);
  //! [filtering]
  conf_map = wls_filter->getConfidenceMap();

  // Get the ROI that was used in the last filter call:
  ROI = wls_filter->getROI();
  if(!no_downscale)
  {
    // upscale raw disparity and ROI back for a proper comparison:
    resize(left_disp,left_disp,cv::Size(),2.0,2.0,cv::INTER_LINEAR);
    left_disp = left_disp*2.0;
    ROI = cv::Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
  }


  cv::ximgproc::getDisparityVis(filtered_disp,disp_map,vis_mult);
  cv::imwrite("disparity_map.png",disp_map);
  cv::Mat raw_disp_vis;
  cv::ximgproc::getDisparityVis(left_disp,raw_disp_vis,vis_mult);
  cv::imwrite("disp_raw.png",raw_disp_vis);
  cv::imwrite("disp_conf.png",conf_map);

}

void Reconstruct::getPointCloud() {
  //cv::Mat Q = (cv::Mat_<double>(4,4) << 1, 0, 0, -564.371521, 0, 1, 0, -385.779392, 0, 0, 0, 682.41307871, 0, 0, 0.1590, 0);//02_26-23 images
  //Mat Q = (cv::Mat_<double>(4,4) << 1, 0, 0, -539.71264, 0, 1, 0, -374.515667, 0, 0, 0, 611.5856, 0, 0, 0.15875653, 0);//03_01-less images
  cv::Mat XYZ(disp_map.size(),CV_32FC3);
  reprojectImageTo3D(disp_map, XYZ, matrix_Q, false, CV_32F);
  std::cout << "xyz size:" << XYZ.size() << std::endl;
  pcl::PointXYZ pt;
  int num=0;
  for (int i=150;i<630;i++) {//XYZ.rows
    for (int j=210;j<870;j++) {//XYZ.cols
      pt.x = (float) XYZ.at<cv::Vec3f>(i,j)[0]/45;
      pt.y = (float) XYZ.at<cv::Vec3f>(i,j)[1]/45;
      pt.z = (float) XYZ.at<cv::Vec3f>(i,j)[2]/45;
      //if ((pt.x>200)||(pt.y>200)||(pt.z>200))
      //continue;
      num++;
      cloud->points.push_back(pt);
    }
  }
  pcl::PCDWriter writer;
  cloud->width = num;
  cloud->height = 1;
  writer.write("cloud_res.pcd",*cloud,1);

  /*
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (auto& point : cloud->points) {
    if ((point.x>-0.6)&&(point.x<0.6)&&(point.y>-1)&&(point.y<1)&&(point.z>0)&&(point.z<4.5))//(point.y>-0.6)&&(point.y<0.25)
      temp_cloud->points.push_back(point);
  }
  temp_cloud->width = temp_cloud->points.size();
  temp_cloud->height = 1;
  */
}

void Reconstruct::getRectifiedImage() {
  cv::Size imageSize;

  cv::Mat M1,D1,R1,P1,M2,D2,R2,P2;
  cv::FileStorage fs;
  fs.open("/home/zoe/learnpc/test_v8/build/extrinsics.yml", cv::FileStorage::READ);
  if( fs.isOpened() )
  {
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> matrix_Q;
    fs.release();
  }
  else
    std::cout << "Error: can not open the extrinsic parameters\n";


  fs.open("/home/zoe/learnpc/test_v8/build/intrinsics.yml", cv::FileStorage::READ);
  if( fs.isOpened() )
  {
    fs["M1"] >> M1;
    fs["M2"] >> M2;
    fs["D1"] >> D1;
    fs["D2"] >> D2;
    fs.release();
  }
  else
    std::cout << "Error: can not open the extrinsic parameters\n";


  imageSize = left_img.size();
  //Precompute maps for cv::remap()
  initUndistortRectifyMap(M1, D1, R1, P1, imageSize,  CV_32FC1, rmap[0][0], rmap[0][1]);
  initUndistortRectifyMap(M2, D2, R2, P2, imageSize,  CV_32FC1, rmap[1][0], rmap[1][1]);

  remap(left_img, left_rect, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
  cvtColor(left_rect, left_rect, cv::COLOR_GRAY2BGR);
  std::cout << "rmap[0][0]:" << rmap[0][0].size() << "  rmap[0][1]" << rmap[0][1].size() << std::endl;
  std::stringstream imageName;
  imageName << "rect_image_left"<<".png";
  imwrite(imageName.str(),left_rect);

  remap(right_img, right_rect, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
  cvtColor(right_rect, right_rect, cv::COLOR_GRAY2BGR);
  //std::cout << "rmap[1][0]:" << rmap[0][0] << "  rmap[1][1]" << rmap[0][1] << std::endl;
  std::stringstream imageName2;
  imageName2 << "rect_image_right"<<".png";
  imwrite(imageName2.str(),right_rect);
}

void Reconstruct::getObjectsVisual() {
  Eigen::Matrix4f transform_matrix = Utils::cloudPCATransform(cloud,cloud_vis);
  std::cout << "visualization cloud size:" <<cloud_vis->points.size() << std::endl;
  std::vector<cv::Point> marker_img,marker_depth;
  for (int i=0;i<obs.size();i++) {
    float min_dist,temp_dist;
    int num = 0,row,col;
    min_dist = Utils::SqDist(obs[i],cloud_vis->points[0]);
    for (int count=1;count<cloud_vis->points.size();count++) {
      temp_dist = Utils::SqDist(obs[i],cloud_vis->points[count]);
      if (temp_dist < min_dist) {
        min_dist = temp_dist;
        num = count;
      }
    }
    std::cout << "   minimum dist:" << min_dist << std::endl;
    col = num%660;
    row = num/660;
    cv::Point obs_temp(col,row),obs_ori_pos;
    obs_img.push_back(obs_temp);
    std::cout << "obstacle-"<<i<<" count:"<<num<< " row:"<<row<<"col:"<<col<<std::endl;
    cv::Point pt1(150+col,90+row),pt2(270+col,210+row);
    marker_depth.push_back(pt1);
    marker_depth.push_back(pt2);
    //rectangle(disp_map,pt1,pt2,254,3);

    obs_ori_pos.x = rmap[0][0].at<float>(obs_temp.y,obs_temp.x)+30;
    obs_ori_pos.y = rmap[0][1].at<float>(obs_temp.y,obs_temp.x)+20;
    std::cout << "   obs_ori_pos:" << obs_ori_pos << std::endl;
    //cv::Point pt_l(140+obs_ori_pos.x,70+obs_ori_pos.y),pt_r(290+obs_ori_pos.x,220+obs_ori_pos.y);
    cv::Point pt_l,pt_r;
    if (obs_ori_pos.x+215<430) {
      pt_l.x = 160+obs_ori_pos.x;
      pt_l.y = 70+obs_ori_pos.y;
      pt_r.x = 310+obs_ori_pos.x;
      pt_r.y = 220+obs_ori_pos.y;
    }
    else if (obs_ori_pos.x+215>720) {
      pt_l.x = 160+obs_ori_pos.x;
      pt_l.y = 70+obs_ori_pos.y;
      pt_r.x = 310+obs_ori_pos.x;
      pt_r.y = 220+obs_ori_pos.y;
    }
    else {
      pt_l.x = 160+obs_ori_pos.x;
      pt_l.y = 70+obs_ori_pos.y;
      pt_r.x = 310+obs_ori_pos.x;
      pt_r.y = 220+obs_ori_pos.y;
    }
    marker_img.push_back(pt_l);
    marker_img.push_back(pt_r);
    //rectangle(left_img_color,pt_l,pt_r,cv::Scalar(0,255,0),3);
    //circle(left_img, obs_ori_pos, 8, (0,0,255), 2);
  }
  rmOverlapped(marker_img,overlappedIdx);
  std::cout << "marker_img size:" << marker_img.size() << std::endl;
  for (int i=0;i<marker_img.size()/2;i++) {
    rectangle(left_img_color,marker_img[2*i],marker_img[2*i+1],cv::Scalar(0,255,0),3);
  }
  cv::imwrite("disp_map_obs_marked.png",disp_map);
  cv::imwrite("left_img_obs_marked.png",left_img_color);
}

//find the overlapped pair of marking rectangles(if any), remove it from the vector of squares and provide the index
void Reconstruct::rmOverlapped(std::vector<cv::Point>& marker, std::vector<int>& overlappedIdx) {
  for (int i=0;i<marker.size()/2-1;i++) {
    for (int j=i+1;j<marker.size()/2;j++) {
      float dist = pow(marker[2*i].x-marker[2*j].x,2) + pow(marker[2*i].y-marker[2*j].y,2);
      dist = pow(dist,0.5);
      std::cout << dist << std::endl;
      //std::cout << i << " " << j << std::endl;
      //std::cout <<obs[i].z << " " << obs[j].z << std::endl;
      if (dist<0.5*150) {
        if (obs[i].z<obs[j].z)
          overlappedIdx.push_back(j);
        else
          overlappedIdx.push_back(i);
      }
    }
  }
  //remove it from the rectangles
  if (overlappedIdx.size()>0) {
    std::sort(overlappedIdx.begin(), overlappedIdx.end(), [](int a, int b) {
          return a > b;
      });
    for (int i=0;i<overlappedIdx.size()-1;i++) {
      //std::cout << overlappedIdx[i] << std::endl;
      if (overlappedIdx[i] == overlappedIdx[i+1])
        overlappedIdx.erase(overlappedIdx.begin()+i+1);
    }
    for (int i=0;i<overlappedIdx.size();i++) {
      marker.erase(marker.begin() + 2*overlappedIdx[i]+1);
      marker.erase(marker.begin() + 2*overlappedIdx[i]);
    }
    std::cout << "marker afterwards:" << std::endl;
    for (int i=0;i<marker.size();i++)
      std::cout << marker[i] << " ";
    std::cout << std::endl;
  }

}
