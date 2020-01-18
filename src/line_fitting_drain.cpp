#include <stdlib.h>
#include <line_fitting_drain.h>
#include <utils.h>

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>

void LineRecogDrain::setInputCloud(std::string name) {
  pcl::PCDReader reader;
  reader.read (name, *cloud_ori);
}

void LineRecogDrain::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr this_cloud) {
  *cloud_ori = *this_cloud;
}

void LineRecogDrain::getSegsAndKeyPts() {
  //cast the points to xy plane (z=0),erase the noise (small clusters far from the main cluster)
  LineRecogDrain::cloudTransform(cloud_ori,cloud);
  //read in shape definers
  if (!LineRecogDrain::readShapeDefiners()) {
    std::cout << "reading in shape definers fails" << std::endl;
    exit(1);
  }
  //get inital point
  LineRecogDrain::getInitialPoint();
  std::cout << "start to get points" << std::endl;
  int num=1;
  for (int i=0;i<2;i++) {
    LineRecogDrain::getFollowingPoint(key_pts[i],shape_definer[i][0],shape_definer[i][1],num);
    num++;
    std::cout << "getting point "<<i<< std::endl;
  }
  key_pts.push_back(end_point);
  for (int i=2;i<4;i++) {
    LineRecogDrain::getFollowingPoint(key_pts[i+1],shape_definer[i][0],shape_definer[i][1],num);
    num++;
    std::cout << "getting point "<<i<< std::endl;
  }
  std::swap(key_pts[3],key_pts[5]);
  LineRecogDrain::getPtElonged(key_pts[4],key_pts[5],key_pts[5]);
  LineRecogDrain::getPtElonged(key_pts[1],key_pts[0],key_pts[0]);

  pcl::PointCloud<pcl::PointXYZ> rect;
  rect =LineRecogDrain::getRect(key_pts[2],key_pts[3],0.03);
  int count = 0;
  for (auto& point : cloud->points) {
    if (Utils::isInside(rect,4,point))
      count++;
  }
  std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&count: " << count << std::endl;
  if (count < 300) {
    key_pts[2].x -= 0.05;
    key_pts[3].x -= 0.05;
  }
  /*
  //get last two points by symmetry
  pcl::PointXYZ pt_temp;
  pt_temp = getSynmatricPt(key_pts[1],key_pts[2],key_pts[3]);
  key_pts.push_back(pt_temp);
  pt_temp = getSynmatricPt(key_pts[0],key_pts[2],key_pts[3]);
  key_pts.push_back(pt_temp);
  */
}

//get starting and ending points for the shape
void LineRecogDrain::getInitialPoint() {
  //std::cout << "cloud point size is "<< cloud->points.size()<<std::endl;
  std::sort (cloud->points.begin(), cloud->points.end(), Utils::smallerPointY);
  y_smaller = cloud->points[0].y;
  y_bigger = cloud->points[cloud->points.size()-1].y;
  pcl::PointXYZ temp_point(0,0,0);
  for (int i=0;i<10;i++)
    temp_point.getVector3fMap() += cloud->points[i].getVector3fMap();
  temp_point.getVector3fMap() /= 10;
  key_pts.clear();
  //correct the x value of the initial point by averaging it with x_medium of this small section;
  float thresh_low,thresh_high,x_average = 0;
  int pt_count = 0;;
  thresh_low = (float)(y_bigger-y_smaller)/5*0.3+y_smaller;
  thresh_high = (float)(y_bigger-y_smaller)/5*0.7+y_smaller;
  for (int i=0;i<cloud->points.size();i++) {
    if ((cloud->points[i].y < thresh_high) && (cloud->points[i].y > thresh_low)) {
      x_average += cloud->points[i].x;
      pt_count++;
    }
  }
  x_average /= pt_count;
  if ((fabs(temp_point.x-x_average)>0.03) && (temp_point.x>x_average))
    temp_point.x = x_average;
  else {
    if (temp_point.x<x_average)
      temp_point.x = 0.7*x_average + 0.3*temp_point.x;
    if (temp_point.x>x_average)
      temp_point.x = 0.3*x_average + 0.7*temp_point.x;
  }
  std::cout << "  temp_point.x:"<< temp_point.x << " x_average:" << x_average << std::endl;
  key_pts.push_back(temp_point);
  //get end point
  temp_point = pcl::PointXYZ(0,0,0);
  for (int i=cloud->points.size()-1;i>cloud->points.size()-21;i--)
    temp_point.getVector3fMap() += cloud->points[i].getVector3fMap();
  temp_point.getVector3fMap() /= 20;
  thresh_low = y_bigger-(float)(y_bigger-y_smaller)/5*0.7;
  thresh_high = y_bigger-(float)(y_bigger-y_smaller)/5*0.3;
  x_average = 0;
  pt_count = 0;
  for (int i=0;i<cloud->points.size();i++) {
    if ((cloud->points[i].y < thresh_high) && (cloud->points[i].y > thresh_low)) {
      x_average += cloud->points[i].x;
      pt_count++;
    }
  }
  if (pt_count>0)
    x_average /= pt_count;
  std::cout << temp_point.x << " -------------the value in the average is:" <<x_average<< std::endl;
  if ((fabs(temp_point.x-x_average)>0.03) && (temp_point.x>x_average))
    temp_point.x = x_average;
  else {
    if (temp_point.x<x_average)
      temp_point.x = 0.7*x_average + 0.3*temp_point.x;
    if (temp_point.x>x_average)
      temp_point.x = 0.3*x_average + 0.7*temp_point.x;
  }
  end_point = temp_point;
}

void LineRecogDrain::getFollowingPoint(pcl::PointXYZ& point_pre,int lb,int hb,int num_pt) {
  pcl::PointXYZ point_end_bound;
  pcl::PointCloud<pcl::PointXYZ> rect;
  //get the cloud cutted for fitting
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cut (new pcl::PointCloud<pcl::PointXYZ>);
  float thresh;
  int pt_count=0;
  if ((num_pt == 1)||(num_pt == 2)) {
    thresh = (float)(y_bigger-y_smaller)/5*(num_pt+0.5)+y_smaller;
    for (int i=0;i<cloud->points.size();i++) {
      if (cloud->points[i].y < thresh) {
        cloud_cut->push_back(cloud->points[i]);
        pt_count++;
      }
    }
    cloud_cut->width = pt_count;
    cloud_cut->height = 1;
  }
  else if ((num_pt == 3)||(num_pt == 4)) {
    thresh = y_bigger -(float)(y_bigger-y_smaller)/5*(num_pt-2+0.3);
    for (int i=0;i<cloud->points.size();i++) {
      if (cloud->points[i].y > thresh) {
        cloud_cut->push_back(cloud->points[i]);
        pt_count++;
      }
    }
    cloud_cut->width = pt_count;
    cloud_cut->height = 1;
  }


  pcl::PCDWriter writer;
  std::stringstream mmm;
  mmm<< "cloud_cut"<<num_pt<<".pcd"<<std::endl;
  writer.write("cloud_cut.pcd",*cloud_cut);


  //std::cout << "  num_pt:" << num_pt<< " cloud_cut size:" << cloud_cut->points.size() << " threshold:" << thresh << " " << y_smaller << " "<< y_bigger<<std::endl;
  int max_deg = LineRecogDrain::findSlopeWithinRange(cloud_cut,point_pre,lb,hb);
  std::cout << "the degree with maximun number of points:" << max_deg << std::endl;
  //get the end point for the line segment
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
  point_end_bound = LineRecogDrain::getTestEndBound(point_pre,max_deg);
  rect =LineRecogDrain::getRect(point_pre,point_end_bound,0.03);

  std::vector<pcl::PointXYZ> forRectDraw;
  for (auto& point : rect.points)
    forRectDraw.push_back(point);
  rects.push_back(forRectDraw);

  for (auto& point : cloud_cut->points) {
    if (Utils::isInside(rect,4,point))
      temp->push_back(point);
  }
  LineRecogDrain::getCurrentCluster(temp,temp,point_pre);

  LineRecogDrain::sortAlongOritation(temp,max_deg);

  pcl::PointXYZ pt(0,0,0);
  int num;
  std::cout << "next point:temp point size:" << temp->points.size() << std::endl;
  if ((temp->points.size()>10) && ((num_pt==1)||(num_pt==3)))
    num = temp->points.size()/3.5;
  else if ((temp->points.size()>10) && ((num_pt==2)||(num_pt==4)))
    num = temp->points.size()/2.5;
  else
    num = 1;
  for (int count=0;count<num;count++) {
    pt.getVector3fMap() += temp->points[count].getVector3fMap();
  }
  pt.getVector3fMap() /= num;
  key_pts.push_back(pt);
}

void LineRecogDrain::cloudTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
  Utils::downsample(cloud_in,cloud_out,0.002);
  Utils::eraseNoise(cloud_out,cloud_out,0.02);
  //addded
  float x_at_larger_y,x_at_smaller_y;
  for (int i=1;i<cloud_out->points.size();i++) {
    if (fabs(cloud_out->points[i].y-0.25)<0.003)
      x_at_larger_y = cloud_out->points[i].x;
    if (fabs(cloud_out->points[i].y-0.05)<0.003)
      x_at_smaller_y = cloud_out->points[i].x;
  }
}

pcl::PointCloud<pcl::PointXYZ> LineRecogDrain::getRect(pcl::PointXYZ& pt_beg, pcl::PointXYZ& pt_end, float width)
{
  pcl::PointXYZ pt_temp;
  pcl::PointCloud<pcl::PointXYZ> pts;
  pt_end.z = pt_beg.z;
  Eigen::Vector3f gradient (0,0,0);
  if (fabs((pt_end.y-pt_beg.y)/(pt_end.x-pt_beg.x))<0.01) {
    pt_temp.x = pt_beg.x;
    pt_temp.y = pt_beg.y+width/2;
    pts.push_back(pt_temp);
    pt_temp.y -= width;
    pts.push_back(pt_temp);
    pt_temp.x = pt_end.x;
    pt_temp.y = pt_end.y-width/2;
    pts.push_back(pt_temp);
    pt_temp.y +=width;
    pts.push_back(pt_temp);
    return pts;
  }
  gradient[1] = -(pt_end.x-pt_beg.x)/(pt_end.y-pt_beg.y);
  gradient[0] = 1/pow((pow(gradient[1],2)+1.0),0.5);
  gradient[1] *= gradient[0];
  pt_temp.getVector3fMap() = pt_beg.getVector3fMap() + gradient*width/2;
  pts.push_back(pt_temp);
  pt_temp.getVector3fMap() -= gradient*width;
  pts.push_back(pt_temp);
  pt_temp.getVector3fMap() = pt_end.getVector3fMap() - gradient*width/2;
  pts.push_back(pt_temp);
  pt_temp.getVector3fMap() += gradient*width;
  pts.push_back(pt_temp);
  return pts;
}

pcl::PointXYZ LineRecogDrain::getTestEndBound(pcl::PointXYZ& point_pre,int deg) {
  if (deg<0)
    deg = deg%360+360;
  pcl::PointXYZ point_end_bound(0,0,0);
  int a = abs(deg+45)/180%2;
  if ((abs(deg)%180 > 45) && (abs(deg)%180 < 135)) {
    point_end_bound.x = point_pre.x+(1*pow(-1,a));
    point_end_bound.y = point_pre.y+(float)1/tan(((float)deg/360)*2*M_PI)*point_end_bound.x;
  }
  else {
    point_end_bound.y = point_pre.y+(1*pow(-1,a));
    point_end_bound.x = point_pre.x + tan(((float)deg/360)*2*M_PI)*point_end_bound.y;
  }
  return point_end_bound;
}

bool LineRecogDrain::readShapeDefiners() {
  std::ifstream myfile;
  myfile.open("/home/zoe/learnpc/test_v8/build/angles.txt");
  std::vector<int> bounds(2);
  if (myfile.is_open()) {
    while ( myfile >> bounds[0] >> bounds[1] )
      shape_definer.push_back(bounds);
  }
  else return false;
  return true;
}

int LineRecogDrain::findSlopeWithinRange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ& point_pre,int lb,int hb) {
  pcl::PointXYZ point_end_bound;
  pcl::PointCloud<pcl::PointXYZ> rect;
  int max_count = 0;
  int max_deg = 0;
  //get the slope
  for (int deg=lb;deg<=hb;deg++) {
    int count =0;
    point_end_bound = LineRecogDrain::getTestEndBound(point_pre,deg);
    rect =LineRecogDrain::getRect(point_pre,point_end_bound,0.01);
    for (auto& point : cloud_in->points) {
      if (Utils::isInside(rect,4,point))
        count++;
    }
    if (count > max_count) {
      max_count = count;
      max_deg = deg;
    }
  }
  return max_deg;
}

void LineRecogDrain::sortAlongOritation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,int max_deg) {
  max_deg%=360;
  bool (*fcnPtr)(pcl::PointXYZ& pt1,pcl::PointXYZ& pt2);
  if (max_deg<0)
    max_deg+=360;
  if (((max_deg>=0)&&(max_deg<45))||((max_deg>=270)&&(max_deg<360)))
    fcnPtr = Utils::largerPointY;
  else if ((max_deg>=45)&&(max_deg<135))
    fcnPtr = Utils::largerPointX;
  else if ((max_deg>=135)&&(max_deg<225))
    fcnPtr = Utils::smallerPointY;
  else
    fcnPtr = Utils::smallerPointX;
  std::sort (cloud_in->points.begin(), cloud_in->points.end(), fcnPtr);//boost::bind(&LineRecogDrain::largerPointY, this, _1, _2)
}

void LineRecogDrain::getCurrentCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,\
                                       pcl::PointXYZ point_pre) {
  //clustering methods
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.15);
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (2000);
  ec.setSearchMethod (tree);

  std::vector<pcl::PointIndices> cluster_indices_line;
  ec.setInputCloud (cloud_in);
  ec.extract (cluster_indices_line);

  if (cluster_indices_line.size()>1) {
      std::cout << "   cluster size >1 for the line recognition: " << cluster_indices_line.size() << " "<< cluster_indices_line[0].indices.size() << " " << cluster_indices_line[1].indices.size()<< std::endl;
      //find which section do we need
      int sect =0;
      bool found = false;
      for (int i=0;i<cluster_indices_line.size();i++) {
        for (auto& point_indice : cluster_indices_line[i].indices) {
          if (Utils::SqDist(cloud_in->points[point_indice],point_pre) < 0.0001) {//find a close point->the point_pre belongs to this cluster
            sect = i;
            found = true;
            break;
          }
        }
        if (found == true)
          break;
      }
      //copy the points current section
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_line (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = cluster_indices_line[sect].indices.begin (); pit != cluster_indices_line[sect].indices.end (); ++pit) {
          temp_line->points.push_back (cloud_in->points[*pit]); //*
          temp_line->width = cluster_indices_line[0].indices.size();
          temp_line->height = 1;
      }
      *cloud_out = *temp_line;
  }
  else
    *cloud_out = *cloud_in;
}

pcl::PointXYZ LineRecogDrain::getSynmatricPt(pcl::PointXYZ& pt_in,pcl::PointXYZ& pt1, pcl::PointXYZ& pt2) {
  float slope = (float)(pt2.y-pt1.y)/(pt2.x-pt1.x);
  pcl::PointXYZ pt_med,pt_line_end,pt_on_line,pt_project,pt_temp(0,0.1,0);
  pt_med.x = (pt1.x+pt2.x)/2;
  pt_med.y = (pt1.y+pt2.y)/2;
  pt_med.z = pt1.z;
  pt_med.getVector3fMap() = (pt_med.getVector3fMap()+pt_temp.getVector3fMap())/2;
  pt_med.z = pt1.z;
  //std::cout << "pt_med:" << pt_med << std::endl;
  pt_line_end.x = pt_med.x+1;
  pt_line_end.y = pt_med.y;
  pt_line_end.z = pt_med.z;
  //std::cout << "pt_line_end:"<< pt_line_end << std::endl;
  Utils::pt2LineProject(pt_in,pt_on_line,pt_med,pt_line_end);
  //std::cout << "pt_on_line:"<< pt_on_line << std::endl;
  pt_project.getVector3fMap() = pt_on_line.getVector3fMap() + (pt_on_line.getVector3fMap()-pt_in.getVector3fMap());
  //std::cout << "pt_project:"<< pt_project << std::endl;
  return pt_project;
}

void LineRecogDrain::getPtElonged(pcl::PointXYZ& pt_beg,pcl::PointXYZ& pt_end,pcl::PointXYZ& pt_out) {//1,0 or 4,5
  pt_out = pt_end;
  if (fabs(pt_beg.x-pt_end.x)<0.005)
    pt_out.y = pt_beg.y<pt_end.y? pt_end.y+0.1 : pt_end.y-0.1;
  else {
    pt_out.y = pt_beg.y<pt_end.y? pt_out.y+0.1 : pt_out.y-0.1;
    pt_out.x = pt_beg.x - (pt_beg.y-pt_out.y)*(float)(pt_beg.x-pt_end.x)/(pt_beg.y-pt_end.y);
  }
}
