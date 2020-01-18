#include <pcl/common/common.h>
#include <ground_extract.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <utils.h>
#include <fstream>


void GrdExt::setInput(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori,std::vector<pcl::PointXYZ>& key_pts,Eigen::Matrix4f& transform) {
  this->cloud_ori = cloud_ori;
  this->key_pts = key_pts;
  key_pts_filter = key_pts;
  float increa;
  if ((key_pts[3].y-key_pts[2].y) < 0.24)
    increa = 0.02;
  else
    increa = 0.05;
    for (int i=1;i<3;i++)
      key_pts_filter[i].y += increa;
    for (int i=3;i<5;i++)
      key_pts_filter[i].y -= increa;
  this->transform_matrix = transform;
  Utils::downsample(cloud_ori,cloud,0.008);
}

void GrdExt::getDescribers(int degree,bool clamped,int ctrl_num_z) {
  this->degree = degree;
  this->ctrl_num_z = ctrl_num_z;
  this->clamped = clamped;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);

  ctrl_num_y = key_pts.size();
  if (degree >= ctrl_num_y)
    degree = ctrl_num_y -1;

  GrdExt::geneFrSingleRow();
  knots_y = create_knots(degree,ctrl_num_y,clamped);
  knots_z = create_knots(degree,ctrl_num_z,clamped);
  //  int get_sample_interval(std::vector<int>& knots,int ctrl_pts_num, int degree, bool clamped)
  samp_interval_y = getSampleInterval(knots_y,ctrl_num_y,degree,clamped);
  samp_interval_z = getSampleInterval(knots_z,ctrl_num_z,degree,clamped);
}

void GrdExt::fixObstacles(std::vector<int>& overlappedIdx) {
  for (int i=0;i<overlappedIdx.size();i++) {
    obstacles.erase(obstacles.begin() + overlappedIdx[i]);
  }
}

void GrdExt::groundRemove() {
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  float y_min = key_pts[0].y, y_max = key_pts[key_pts.size()-1].y;
  for (int pt=0;pt<cloud->points.size();pt++) {
    if ((cloud->points[pt].y < y_max) && (cloud->points[pt].y > y_min)\
    && (cloud->points[pt].z < max_pt.z) && (cloud->points[pt].z > min_pt.z)) {
      pcl::PointXYZ pt_temp(0.0,0.0,0.0);
      float t_z = (cloud->points[pt].z-min_pt.z)/(max_pt.z-min_pt.z)*samp_interval_z;
      float t_y = GrdExt::getTy(cloud->points[pt],key_pts_filter);
      //float t_y = (cloud->points[pt].y-y_min)/(y_max-y_min)*samp_interval_y;
      if (clamped == false) {
        t_z = t_z+degree;
        t_y = t_y+degree;
      }

      for (size_t j=0;j<ctrl_num_z;j++) {
        for (size_t i=0;i<ctrl_num_y;i++) {
          float sp_basic = spline_bfunc(t_y,i,degree+1,knots_y)*spline_bfunc(t_z,j,degree+1,knots_z);
          pt_temp.getVector3fMap() += ctrl_pts[j][i].getVector3fMap()* sp_basic;
        }
      }
      if (fabs(pt_temp.x-cloud->points[pt].x)<0.037) //origin one <0.03
      inliers -> indices.push_back(pt);
    }
  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_aft_rm);
}

void GrdExt::calculateObstacles() {
  GrdExt::getCloudsResult();

  int outside = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  float y_min = key_pts[0].y, y_max = key_pts[key_pts.size()-1].y;
  for (int m=0;m<clouds_result.size();m++) {
    outside = 0;
    *cloud_temp = clouds_result[m];

    //make sure that the lowest point of cluster is not floating in the air
    if (GrdExt::objectFloat(cloud_temp))
      continue;

    for (int u=0;u<clouds_result[m].points.size();u++) {
      if ((clouds_result[m].points[u].y < y_max) && (clouds_result[m].points[u].y > y_min)\
         && (clouds_result[m].points[u].z < max_pt.z) && (clouds_result[m].points[u].z > min_pt.z)) {
        pcl::PointXYZ pt_project(0.0,0.0,0.0);
        float t_z = (clouds_result[m].points[u].z-min_pt.z)/(max_pt.z-min_pt.z)*samp_interval_z;
        float t_y = GrdExt::getTy(clouds_result[m].points[u],key_pts);
        if (clamped == false) {
          t_z = t_z+degree;
          t_y = t_y+degree;
        }
        for (size_t j=0;j<ctrl_num_z;j++) {
        for (size_t i=0;i<ctrl_num_y;i++) {
          float sp_basic = spline_bfunc(t_y,i,degree+1,knots_y)*spline_bfunc(t_z,j,degree+1,knots_z);
          pt_project.getVector3fMap() += ctrl_pts[j][i].getVector3fMap()* sp_basic;
        }
      }
        cloud_temp->points.push_back(pt_project);
      }
      else
        outside++;
      if (outside>10)
        continue;
    }
    if (outside > 10) {
      continue;
    }

    std::cout << "cloud_temp size:" << m << " " << cloud_temp->points.size() << std::endl;
    if (!GrdExt::initialObjectCheck(cloud_temp))
      continue;
    std::cout << "cloud_temp size:" << m << " " << cloud_temp->points.size() << std::endl;
    if (GrdExt::getVolPolyg(cloud_temp))
      clouds_result[m]=*cloud_temp;
  }
  std::cout << "amends_result size:" << amends_result.size() << std::endl;
  for (int i=0;i<amends_result.size();i++) {
    //std::cout << "here " << i << " " << amends_result[i].points.size() <<  std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempp (new pcl::PointCloud<pcl::PointXYZ>);
    *tempp = amends_result[i];
    if (GrdExt::getVolPolyg(tempp))
      clouds_result.push_back(*tempp);
  }
}

bool GrdExt::getVolPolyg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {
  GrdExt::Obstacles temp_ob;
  float area;
  //get the convex hull
  pcl::ConvexHull<pcl::PointXYZ> cHull;
  cHull.setComputeAreaVolume(true);
  cHull.setInputCloud(cloud_in);
  cHull.reconstruct (temp_ob.cHull_points,temp_ob.polyg);
  temp_ob.volume = cHull.getTotalVolume();
  temp_ob.centroid = getCentroid(temp_ob.cHull_points);
  //make sure the object is not on the slopes
  if (((temp_ob.centroid.y > key_pts[1].y) && (temp_ob.centroid.y < key_pts[2].y)) || ((temp_ob.centroid.y > key_pts[3].y) && (temp_ob.centroid.y < key_pts[4].y)))
    return false;

  //make sure the object is not the noise at the edge of the map constructed
  if ((temp_ob.centroid.y > 0.5)||(temp_ob.centroid.z < 0.75))
    return false;
  GrdExt::getLengthWidthHeight(temp_ob);
  //if the obstacle is at the side and is not high, it is probably noise
  float ratio = (float) (pow(temp_ob.depth,2)/temp_ob.width/temp_ob.height);
  //if (((temp_ob.centroid.y < -0.27)||(temp_ob.centroid.y > 0.46)) && ((temp_ob.height<0.09)||(temp_ob.height>0.28)))
  //  return false;

  if (((temp_ob.centroid.y < -0.28)||(temp_ob.centroid.y > 0.48)) && (((temp_ob.height<0.09)&&(ratio > 8)) || ((temp_ob.height>0.4)&&(temp_ob.depth>0.3))))
    return false;
  //if the obstacle recognized is long along z axis, it is probably noise
  if ((ratio > 6) && (temp_ob.height<(float)ratio/100)&&(temp_ob.height < 0.1))
    return false;
  if ((ratio > 50) && (temp_ob.depth/temp_ob.height > 6) && ((temp_ob.centroid.y<key_pts[2].y)||(temp_ob.centroid.y>key_pts[3].y)) && (temp_ob.height < 0.15))
    return false;
  if ((temp_ob.depth/temp_ob.height>3) && (temp_ob.width/temp_ob.height>1.4) && (temp_ob.depth>0.8) && (temp_ob.height<0.15))
    return false;
  if ((temp_ob.centroid.z>2) && (temp_ob.height<0.09))
    return false;
  //if ( (ratio > 30) || (temp_ob.depth/temp_ob.width > 4) || (temp_ob.height/temp_ob.width>1.3) )
  //  return false;
  //if the height too low or volume too large, probably not Obstacles
  if ((temp_ob.height<0.1)||(temp_ob.volume>0.05)||(temp_ob.height>0.45))
    return false;

  //a weird position always found
  if ((temp_ob.centroid.z>1.6) && (temp_ob.centroid.z<2.2) && (temp_ob.centroid.y>-0.1) && (temp_ob.centroid.y<0.22) && ((temp_ob.height<0.09) || ((temp_ob.height<0.12)&&(temp_ob.volume<0.0016))))
    return false;
  //for the object in the middle,further ones
  if ((temp_ob.volume == 0) && (cloud_in->points.size()>200) && (GrdExt::objectOnTop(temp_ob.centroid,temp_ob.cHull_points)) && (temp_ob.height > 0.2) \
  && (temp_ob.centroid.y>key_pts_filter[2].y) && (temp_ob.centroid.y<key_pts_filter[3].y) && (temp_ob.centroid.z > 1.9)) {
    std::cout << "cloud points size:" << cloud_in->points.size() << " width:" << temp_ob.width << " height:" << temp_ob.height;
    std::cout << " coordinate: " << temp_ob.centroid << std::endl;
    temp_ob.volume = temp_ob.width*temp_ob.height*temp_ob.width/2;
    std::cout << "volume:" << temp_ob.volume << std::endl;
    overSize = true;
    obstacles.push_back(temp_ob);
    return true;
  }
  //other objects
  if ((temp_ob.volume>0.0002) && (GrdExt::objectOnTop(temp_ob.centroid,temp_ob.cHull_points))) {  //((temp_ob.volume > 0.0002)&&(GrdExt::objectOnTop(temp_ob.centroid,temp_ob.cHull_points)))
    if ((temp_ob.height>0.18) && (temp_ob.centroid.y>key_pts[2].y) && (temp_ob.centroid.y<key_pts[3].y))
      overSize = true;
    std::cout << "  ratio:" << ratio << " height:" << temp_ob.height << std::endl;
    std::cout << "the volume of the object is:" << temp_ob.volume << " m^3." << " width:" \
    << temp_ob.width << " depth:" << temp_ob.depth << " height:" << temp_ob.height<< std::endl;
    std::cout << "coordinate: " << temp_ob.centroid << std::endl;
    //if the object is too long, assume the depth is the same as the width
    float depth_width = temp_ob.depth/temp_ob.width;
    if (depth_width > 2.5) {
      temp_ob.centroid.z -= (temp_ob.depth-temp_ob.width)/3;
      temp_ob.depth /= depth_width;
      temp_ob.volume /= depth_width;
      //too close, must be bulk noise
      if (temp_ob.centroid.z < 0.8)
        return false;
      std::cout << "afterwards ratio:" << ratio << " height:" << temp_ob.height << std::endl;
      std::cout << "the volume of the object is:" << temp_ob.volume << " m^3." << " width:" \
      << temp_ob.width << " depth:" << temp_ob.depth << " height:" << temp_ob.height<< std::endl;
    }
    obstacles.push_back(temp_ob);
    return true;
  }
  else {
    //obstacles.push_back(temp_ob);
    return false;
  }
}

void GrdExt::getCloudsResult(float toler,int min_clus_size,int max_clus_size) {
  //new added block, declare clustering methods
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_aft_rm);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (toler);
  ec.setMinClusterSize (min_clus_size);
  ec.setMaxClusterSize (max_clus_size);
  ec.setSearchMethod (tree);

  std::vector<pcl::PointIndices> cluster_indices;
  ec.setInputCloud (cloud_aft_rm);
  ec.extract (cluster_indices);

  clouds_result.resize(cluster_indices.size());
  int num=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      clouds_result[num].points.push_back (cloud_aft_rm->points[*pit]); //*
    num++;
  }
}

void GrdExt::geneFrSingleRow() {
  float ctrl_z = min_pt.z;
  ctrl_pts.resize(ctrl_num_z, std::vector<pcl::PointXYZ>(ctrl_num_y));
  for (int j=0;j<ctrl_num_z;j++) {
    for (int i=0;i<ctrl_num_y;i++) {
      ctrl_pts[j][i].y = key_pts_filter[i].y;
      ctrl_pts[j][i].z = ctrl_z;
      ctrl_pts[j][i].x = key_pts_filter[i].x;
    }
    ctrl_z += (float)(max_pt.z-min_pt.z)/(ctrl_num_z-1);
  }
}

float GrdExt::spline_bfunc(float t, int i, int k,std::vector<int>& knots) {
  //k-order,i-number of control points-1,t-variable(x value)
  if (k == 1)
  {
    if (t >= knots[i] && t < knots[i+1] )
    return 1.0;
    else
    return 0.0;
  }
  else
  {
    float a,b,subweight_a=0,subweight_b=0;
    a = knots[i+k-1]-knots[i];
    b = knots[i+k]-knots[i+1];
    if (a!=0)
    subweight_a = (float)spline_bfunc(t,i,k-1,knots)*(t-knots[i])/a;
    if(b!=0)
    subweight_b = (float)spline_bfunc(t,i+1,k-1,knots)*(knots[i+k]-t)/b;
    return (subweight_a+subweight_b);
  }
}

std::vector<int> GrdExt::create_knots(int degree, int ctl_size, bool clamped) {
  std::vector<int> knots;
  if (clamped == false) {
    for (int i=0;i<degree+ctl_size+1;i++)
    knots.push_back(i);
  }
  else {
    int n=1;
    for (int i=0;i<degree+1;i++)
    knots.push_back(0);
    for (int i = degree+1;i< ctl_size;i++)
    knots.push_back(i-degree);
    for (int i =ctl_size;i<degree+ctl_size+1;i++)
    knots.push_back(ctl_size-degree);
  }
  return knots;
}

int GrdExt::getSampleInterval(std::vector<int>& knots,int ctrl_pts_num, int degree, bool clamped) {
  if (clamped == 1)
  return knots[knots.size()-1];
  else
  return ctrl_pts_num-degree;
}

//for t_y determination, since the key points along y axis are not equally spaced
float GrdExt::getTy(pcl::PointXYZ pt,std::vector<pcl::PointXYZ>& key_pts) {
  for (int i=0;i<key_pts.size()-1;i++) {
    if ((pt.y>=key_pts[i].y) && (pt.y<key_pts[i+1].y))
      return i+(float)(pt.y-key_pts[i].y)/(key_pts[i+1].y-key_pts[i].y);
  }
}

bool GrdExt::objectOnTop(pcl::PointXYZ& centroid,pcl::PointCloud<pcl::PointXYZ>& chull_pts) {
  pcl::PointXYZ pt_line,pt_beg,pt_end;
  int count;
  for (int i=0;i<key_pts.size()-1;i++) {
    if ((centroid.y>=key_pts[i].y) && (centroid.y<key_pts[i+1].y)) {
      pt_beg.getVector3fMap() = key_pts[i].getVector3fMap();
      pt_end.getVector3fMap() = key_pts[i+1].getVector3fMap();
      pt_beg.z = pt_end.z = centroid.z;
      count = i;
      std::cout << "   i=" << i <<std::endl;
      break;
    }
  }
  // if the object on sides, make decision based on the centroid, else, make decision based on the point with lower x
  if ((count == 0) || (count == 4)) {
    float lower_x;
    if (centroid.x > (pt_beg.x+pt_end.x)/2) {
      std::cout << centroid << " " << (pt_beg.x+pt_end.x)/2 << std::endl;
      std::cout << "object on sides, obstacle - true" << std::endl;
      return true;
    }
    else {
      std::cout << centroid << " " << (pt_beg.x+pt_end.x)/2 << std::endl;
      std::cout << "object on sides, obstacle - false" << std::endl;
      return false;
    }
  }
  if (count == 2) {
    int num = 0;
    bool found = false;
    std::sort (chull_pts.points.begin(), chull_pts.points.end(), Utils::smallerPointX);
    for (int i=0;i<chull_pts.size()/2;i++) {
      if (fabs(chull_pts[i].y-centroid.y)<0.04) {
        num = i;
        found = true;
        break;
      }
    }
    Utils::pt2LineProject(chull_pts[num], pt_line, pt_beg, pt_end);
    std::cout << "object on bottom " <<chull_pts.size() << " " << num << " " << centroid << " " << chull_pts.points[num]<< " " << pt_line << std::endl;
    if (chull_pts.points[num].x > (pt_line.x-0.01)) {
      std::cout << "bottom true" << std::endl;
      return true;
    }
    else {
      std::cout << "bottom false" << std::endl;
      return false;
    }
  }
  if ((count == 1) || (count == 3)) {
    pcl::PointXYZ pt_temp;
    Utils::pt2LineProject(centroid,pt_temp,key_pts[count],key_pts[count+1]);
    if (centroid.x > pt_temp.x)
      return true;
    else
      return false;
  }
}

pcl::PointXYZ GrdExt::getCentroid(pcl::PointCloud<pcl::PointXYZ>& cloud) {
  pcl::PointXYZ temp(0,0,0);
  for (auto& point : cloud.points)
    temp.getVector3fMap() += point.getVector3fMap();
  temp.getVector3fMap() /= cloud.points.size();
  return temp;
}

bool GrdExt::objectFloat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {
  std::sort (cloud_in->points.begin(), cloud_in->points.end(), Utils::smallerPointX);
  pcl::PointXYZ x_min(0,0,0),y_med(0,0,0);
  for (int i=0;i<5;i++) {
    x_min.getVector3fMap() += cloud_in->points[i].getVector3fMap();
  }
  x_min.getVector3fMap() /= 5;
  std::sort (cloud_in->points.begin(), cloud_in->points.end(), Utils::smallerPointY);
  y_med.getVector3fMap() = (cloud_in->points[0].getVector3fMap()+cloud_in->points[cloud_in->points.size()-1].getVector3fMap())/2;
  //std::cout << "x_min of cluster:" << x_min<<std::endl;
  //std::cout << "y of cluster:" << y_med<<std::endl;
  int pos = GrdExt::getTy(y_med,key_pts);
  //std::cout << "pos:"<<pos<<std::endl;
  float x_medium = (float) (key_pts[pos].x+key_pts[pos+1].x)/2;
  //std::cout << "the x_medium:" << x_medium << std::endl;
  if ((x_min.x-x_medium) > 0.045)
    return true;
  else
    return false;
}

void GrdExt::getLengthWidthHeight(Obstacles& temp_ob) {
  int size = temp_ob.cHull_points.points.size()-1;
  std::sort (temp_ob.cHull_points.points.begin(), temp_ob.cHull_points.points.end(), Utils::smallerPointX);
  temp_ob.height = temp_ob.cHull_points.points[size].x-temp_ob.cHull_points.points[0].x;
  std::sort (temp_ob.cHull_points.points.begin(), temp_ob.cHull_points.points.end(), Utils::smallerPointY);
  temp_ob.width = temp_ob.cHull_points.points[size].y-temp_ob.cHull_points.points[0].y;
  std::sort (temp_ob.cHull_points.points.begin(), temp_ob.cHull_points.points.end(), Utils::smallerPointZ);
  temp_ob.depth = temp_ob.cHull_points.points[size].z-temp_ob.cHull_points.points[0].z;
}

bool GrdExt::initialObjectCheck(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {
  pcl::PointXYZ centroid;
  pcl::PointCloud<pcl::PointXYZ> cloud_temp;
  centroid = GrdExt::getCentroid(*cloud_in);
  std::sort (cloud_in->points.begin(), cloud_in->points.end(), Utils::smallerPointY);
  float width = cloud_in->points[cloud_in->points.size()-1].y - cloud_in->points[0].y;
  //std::cout << "centroid: " << centroid.y << std::endl;
  if ((((centroid.y > key_pts[1].y) && (centroid.y < key_pts[2].y)) || ((centroid.y > key_pts[3].y) && (centroid.y < key_pts[4].y))) && (width < 0.45))
    return false;
  //if the object is on the bottom
  //if ((centroid.y > key_pts[2].y) && (centroid.y < key_pts[3].y)) {
  if (((centroid.y > key_pts[2].y) && (centroid.y < key_pts[3].y))||(width>0.45)) {
    //if width > 0.45, cut the cloud into 2 parts and push the one by the side to the amends_result
    if (width>0.45) {
      pcl::PointCloud<pcl::PointXYZ> supple;
      if ((cloud_in->points[cloud_in->points.size()-1].y) < key_pts[4].y) {
        for (int i=0;i<cloud_in->points.size()-1;i++) {
          if ((cloud_in->points[i].y < key_pts_filter[1].y))
            supple.points.push_back(cloud_in->points[i]);
        }
      }
      if ((cloud_in->points[cloud_in->points.size()-1].y) > key_pts[4].y) {
        for (int i=0;i<cloud_in->points.size()-1;i++) {
          if ((cloud_in->points[i].y > key_pts_filter[4].y))
            supple.points.push_back(cloud_in->points[i]);
        }
      }
      amends_result.push_back(supple);
    }
    std::cout << "))))))))size: "<< cloud_in->points.size() << std::endl;
    for (int i=0;i<cloud_in->points.size()-1;i++) {
      if ((cloud_in->points[i].y > key_pts_filter[2].y) && (cloud_in->points[i].y < key_pts_filter[3].y) && (cloud_in->points[i].x > -0.24))
        cloud_temp.points.push_back(cloud_in->points[i]);
    }
    *cloud_in = cloud_temp;
    std::cout << "))))))))size: "<< cloud_in->points.size() << std::endl;

    std::cout << cloud_in->points[0] << " " << cloud_in->points[cloud_in->points.size()-1] << std::endl;
    std::sort (cloud_in->points.begin(), cloud_in->points.end(), Utils::smallerPointZ);
    if ((cloud_in->points[0].z+cloud_in->points[cloud_in->points.size()-1].z)/2 < 1.8) {
      if (cloud_in->points.size() < 350)
        return false;
      }
    else {
      if (cloud_in->points.size() < 200)
        return false;
      }
    int k = cloud_in->points.size()/5-1;
    std::cout << "20 percent:" << cloud_in->points[k] << " " << cloud_in->points[k*2] << " " << cloud_in->points[k*3] << " " << cloud_in->points[k*4] << std::endl;
    std::cout << "total diff:" << cloud_in->points[cloud_in->points.size()-1].z - cloud_in->points[0].z << std::endl;
    std::cout << "difference:" << cloud_in->points[cloud_in->points.size()-1].z- cloud_in->points[k*4].z << " " << cloud_in->points[k*4].z - cloud_in->points[k*3].z \
    << " " << cloud_in->points[k*3].z - cloud_in->points[k*2].z << " " << cloud_in->points[k*2].z - cloud_in->points[k].z << " " << cloud_in->points[k].z - cloud_in->points[0].z << std::endl;
    int coun = 0;
    std::sort (cloud_temp.points.begin(), cloud_temp.points.end(), Utils::smallerPointX);
    //height is taller than certain threshold, then true
    if (((cloud_temp.points[cloud_temp.points.size()-1].x - cloud_temp.points[0].x) > 0.24) && (centroid.y>key_pts_filter[2].y) && (centroid.y<key_pts_filter[3].y))
      return true;
    cloud_temp.clear();
    for (int i=0;i<5;i++) {
      std::cout << "i=" << i << std::endl;
    if ((cloud_in->points[k*(i+1)].z-cloud_in->points[k*i].z) < 0.003) {//0.002
      for (int n=0;n<cloud_in->points.size()-1;n++)
        if (cloud_in->points[n].z > (cloud_in->points[k*i].z-0.02))
          cloud_temp.points.push_back(cloud_in->points[n]);
        *cloud_in = cloud_temp;
        return true;
      }
    if (((cloud_in->points[k*(i+1)].z-cloud_in->points[k*i].z) < 0.008) && (cloud_in->points.size()<800))
        coun++;
    if (((cloud_in->points[k*(i+1)].z-cloud_in->points[k*i].z) < 0.03) && (cloud_in->points.size()>800))//0.03
        coun++;
    if (coun>1) {
      for (int n=0;n<cloud_in->points.size()-1;n++)
        if (cloud_in->points[n].z > (cloud_in->points[k*i].z-0.05))
          cloud_temp.points.push_back(cloud_in->points[n]);
        *cloud_in = cloud_temp;
      return true;
     }
    }
    std::fstream fs;
    fs.open ("pts_obstacle.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    for (int i=0;i < cloud_in->points.size()-1;i++) {
      fs << cloud_in->points[i].z << std::endl;
    }
    return false;
  }
  //if the object is on the side
  else {
    if (centroid.y <= key_pts[1].y) {
      float thresh;
      for (int i=0;i<cloud_in->points.size()-1;i++) {
        thresh = cloud_in->points[0].y + 0.2*width;
        if ((cloud_in->points[i].y > thresh) && (cloud_in->points[i].y < key_pts[1].y))
          cloud_temp.push_back(cloud_in->points[i]);
      }
      *cloud_in = cloud_temp;
    }
    else if (centroid.y >= key_pts[4].y) {
      float thresh;
      for (int i=0;i<cloud_in->points.size()-1;i++) {
        thresh = cloud_in->points[4].y + 0.8*width;
        if ((cloud_in->points[i].y < thresh) && (cloud_in->points[i].y > key_pts[4].y))
          cloud_temp.push_back(cloud_in->points[i]);
      }
      *cloud_in = cloud_temp;
    }
    return true;
  }
}
