#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common.h>
#include <cloud_slicing.h>
#include <utils.h>

void CloudSlice::setInputCloud(std::string name,bool downsample) {
  pcl::PCDReader reader;
  reader.read (name, *cloud_ori);
  CloudSlice::limitViewRange(cloud_ori,cloud_ori);
  //transform_matrix = Utils::cloudPCATransform(cloud_ori,cloud_ori);
  if (downsample == true)
    Utils::downsample(cloud_ori,cloud_ori,(float)0.008);
  else
    Utils::rmNAN(cloud_ori,cloud_ori);
  transform_matrix = Utils::cloudPCATransform(cloud_ori,cloud_ori);
  CloudSlice::getSectNum();
}

void CloudSlice::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr this_cloud,bool downsample) {
  *cloud_ori = *this_cloud;
  std::cout <<"cloud_ori size:"<<cloud_ori->points.size()<<std::endl;
  CloudSlice::limitViewRange(cloud_ori,cloud_ori);
  //transform_matrix = Utils::cloudPCATransform(cloud_ori,cloud_ori);
  if (downsample == true)
    Utils::downsample(cloud_ori,cloud_ori,(float)0.008);
  else
    Utils::rmNAN(cloud_ori,cloud_ori);
  transform_matrix = Utils::cloudPCATransform(cloud_ori,cloud_ori);
  CloudSlice::getSectNum();
  std::cout << "finish setting input cloud" <<std::endl;
}

void CloudSlice::setSectRange(float sect_range) {
  CloudSlice::getSectNum();
  this->sect_range = sect_range;
}

void CloudSlice::setSectDist(float sect_dist) {
  this->sect_dist = sect_dist;
  CloudSlice::getSectNum();
}

void CloudSlice::getSectNum() {
  pcl::getMinMax3D(*cloud_ori, min_pt, max_pt);
  //the minimum and maximum has to change a bit
  min_pt.z += 0.05;
  max_pt.z -= 0.05;
  sect_num = (max_pt.z-min_pt.z)/sect_dist;
}

void CloudSlice::getOneSect(float axis_z) {
  cloud_extract.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i=0;i<cloud_ori->points.size();i++) {
    if ((cloud_ori->points[i].z >= (axis_z-sect_range*0.5)) && (cloud_ori->points[i].z < (axis_z+sect_range*0.5)))
      cloud_extract ->points.push_back(cloud_ori->points[i]);
  }
}

void CloudSlice::sectProject(float axis_z) {
  CloudSlice::getOneSect(axis_z);
  //project above points
  coefficient->values[3] = -axis_z;
  Utils::cloud2PlaneProject(cloud_extract,cloud_extract,coefficient);
}

void CloudSlice::limitViewRange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (auto& point : cloud_in->points) {
    if (((point.x>-0.34)&&(point.x<0.53)&&(point.y>-0.2)&&(point.y<0.5)&&(point.z>0.4)&&(point.z<1.9))\
       ||((point.x>-0.32)&&(point.x<0.53)&&(point.y>-0.2)&&(point.y<0.5)&&(point.z>1.9)&&(point.z<2.5))) //(point.y>-0.6)&&(point.y<0.25)//-0.36&-0.34
      temp_cloud->points.push_back(point);
  }
  temp_cloud->width = temp_cloud->points.size();
  temp_cloud->height = 1;
  *cloud_out = *temp_cloud;
}
