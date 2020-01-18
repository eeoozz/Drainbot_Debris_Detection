#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <utils.h>
#include <stdlib.h>

#define inf 10000

Eigen::Matrix4f Utils::cloudPCATransform (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
  /*
  pcl::PointCloud<pcl::PointXYZ>::Ptr prt_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (int i=0;i<cloud_in->points.size();i++) {
    if (cloud_in->points[i].z>1.2)
      prt_cloud->points.push_back(cloud_in->points[i]);
  }
  prt_cloud->width = prt_cloud->points.size();
  prt_cloud->height = 1;
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*prt_cloud, centroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*prt_cloud, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
  eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

  // move the points to the reference frame
  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block<3,3>(0,0) = eigDx.transpose();
  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
  pcl::transformPointCloud(*cloud_in, *cloud_out, p2w);
  Eigen::Matrix4f yk;
  yk << -1,0,0,0,
        0,-1,0,0,
        0,0,1,0,
        0,0,0,1;
  pcl::transformPointCloud(*cloud_out, *cloud_out, yk);

  //for transforming centroids of obstacles back later
  Eigen::Matrix4f mult;
  Eigen::Matrix3f temp;
  mult = (yk*p2w).inverse();
  return mult;
  */

  //*cloud_out = *cloud_in;
  Eigen::Matrix4f yk;
  yk << 0,-1,0,0,
        1,0,0,0,
        0,0,1,0,
        0,0,0,1;
  pcl::transformPointCloud(*cloud_in, *cloud_out, yk);
  Eigen::Matrix4f ym;
  ym << 0.999,  0.0000000,  0.0349, 0,
        0,  1,  0, 0,
        -0.0349,  0,  0.999, 0,
        0, 0, 0, 1;
  pcl::transformPointCloud(*cloud_out, *cloud_out, ym);

  return (ym*yk).inverse();

}

void Utils::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,\
                        float voxel_size) {
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize ((float)voxel_size,(float)voxel_size,(float)voxel_size);
  vg.filter (*cloud_out);

}

void Utils::rmNAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, indices);
}

void Utils::cloud2PlaneProject(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,\
                               pcl::ModelCoefficients::Ptr coefficient) {
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_in);
  proj.setModelCoefficients (coefficient);
  proj.filter (*cloud_out);
  }

void Utils::pt2LineProject(pcl::PointXYZ& pt_in,pcl::PointXYZ& pt_out,pcl::ModelCoefficients& coefficient) {
    //A + dot(AP,AB) / dot(AB,AB) * AB
    Eigen::Vector3f pt_a,ab,ap,p;
    float t=0.2;
    pt_a[0] = coefficient.values[0];
    pt_a[1] = coefficient.values[1];
    pt_a[2] = coefficient.values[2];
    ab[0] = t*coefficient.values[3];
    ab[1] = t*coefficient.values[4];
    ab[2] = t*coefficient.values[5];
    ap = pt_in.getVector3fMap()-pt_a;
    p = pt_a + ap.dot(ab)/ab.dot(ab)*ab;
    pt_out.getVector3fMap() = p;
}

void Utils::pt2LineProject(pcl::PointXYZ& pt_in,pcl::PointXYZ& pt_out,pcl::PointXYZ& pt_begin,pcl::PointXYZ& pt_end) {
  Eigen::Vector3f ab,ap,p;
  ab = pt_end.getVector3fMap() - pt_begin.getVector3fMap();
  ap = pt_in.getVector3fMap() - pt_begin.getVector3fMap();
  p = pt_begin.getVector3fMap() + ap.dot(ab)/ab.dot(ab)*ab;
  pt_out.getVector3fMap() = p;
}

bool Utils::isInVector(int num,std::vector<int> idx) {
  for (int i=0;i<idx.size();i++) {
    if (idx[i] == num)
      return true;
  }
  return false;
}

float Utils::SqDist(pcl::PointXYZ& pt1,pcl::PointXYZ& pt2) {
  Eigen::Vector3f s;
  s = pt1.getVector3fMap()-pt2.getVector3fMap();
  return s.dot(s.transpose());
}

void Utils::eraseNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,float clus_toler) {

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (clus_toler);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (30000);
  ec.setSearchMethod (tree);

  std::vector<pcl::PointIndices> cluster_indices_line;
  ec.setInputCloud (cloud_in);
  ec.extract (cluster_indices_line);

  std::cout << "in eraseNoise, clusters recognized:" << cluster_indices_line.size() << std::endl;
  if (cluster_indices_line.size()==0)
    *cloud_out = *cloud_in;
  else {
    std::vector<int> clusters;
    for (int i=0;i<cluster_indices_line.size();i++) {
      if (cluster_indices_line[i].indices.size()>200) {
        clusters.push_back(i);
      }
    }
    /*
    float low_x=1;
    for (int i=clusters.size()-1;i>-1;i--) {
      for (std::vector<int>::const_iterator pit = cluster_indices_line[clusters[i]].indices.begin (); pit != cluster_indices_line[clusters[i]].indices.end (); ++pit) {
          if (cloud_in->points[*pit].x < low_x)
            low_x = cloud_in->points[*pit].x;
      }
      std::cout << "low_x:"<<low_x << std::endl;
      if (low_x>0.15) {
        clusters.erase(clusters.begin()+i);
        std::cout << "erase " << i << std::endl;
      }
    }
    std::cout << "in eraseNoise, clusters recognized:" << clusters.size() << std::endl;
    */
    int size=0;
    //get rid of other points
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0;i<clusters.size();i++)  {
      for (std::vector<int>::const_iterator pit = cluster_indices_line[clusters[i]].indices.begin (); pit != cluster_indices_line[clusters[i]].indices.end (); ++pit) {
          temp_cloud->points.push_back (cloud_in->points[*pit]); //*
      }
      size+=cluster_indices_line[clusters[i]].indices.size();
    }
    temp_cloud->width = size;
    temp_cloud->height = 1;
    std::cout << "cloud size:" << size<< std::endl;
    *cloud_out = *temp_cloud;
  }

}

bool Utils::onSegment(pcl::PointXYZ& p, pcl::PointXYZ& q, pcl::PointXYZ& r)  {
    if (q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y) &&
            q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x))
        return true;
    return false;
}

int Utils::orientation(pcl::PointXYZ& p, pcl::PointXYZ& q, pcl::PointXYZ& r)  {
    float val = (q.x - p.x) * (r.y - q.y) -(q.y - p.y) * (r.x - q.x);

    if (fabs(val) < 0.000001) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool Utils::doIntersect(pcl::PointXYZ& p1, pcl::PointXYZ& q1, pcl::PointXYZ& p2, pcl::PointXYZ& q2)  {
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}

bool Utils::isInside(pcl::PointCloud<pcl::PointXYZ> polygon, int n, pcl::PointXYZ p)
{
    // There must be at least 3 vertices in polygon[]
    if (n < 3)  return false;

    // Create a point for line segment from p to infinite
    pcl::PointXYZ extreme = {inf, p.y, 0};

    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%n;
        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(polygon[i], polygon[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(polygon[i], p, polygon[next]) == 0)
               return onSegment(polygon[i], p, polygon[next]);

            count++;
        }
        i = next;
    } while (i != 0);

    // Return true if count is odd, false otherwise
    return count&1;  // Same as (count%2 == 1)
}

void Utils::pointTransform(Eigen::Matrix4f& transform, pcl::PointXYZ& point_in,pcl::PointXYZ& point_out) {
  Eigen::Vector4f vec;
  vec.block<3,1>(0,0) = point_in.getVector3fMap();
  vec(3,0) = 1.0;
  vec = transform*vec;
  point_out.getVector3fMap() = vec.block<3,1>(0,0);
}
