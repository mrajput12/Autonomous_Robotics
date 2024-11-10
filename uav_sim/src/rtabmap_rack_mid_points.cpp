/*

This Node is Currentlty NOT TAKING RGB live rtabmap cloud as input instead using saved pcd and generates mid points
- This scripts applies pass-through filter within specific area in "ZXY" axis respectively
- Then voxel grid is created for filtered area to speed up computation
- Planner segmentation of voxelized cloud is Done
- Segmentated plane is utilized to obtained non-planner area -> legs


REMAINING :
  - Live map siving of RTABmap
  - Utilizign map for mid point extraction
TO RUN
    - Make && ./pass_voxel_ransac
TO ViEW
    -  pcl_viewer -multiview 2 ground.pcd legs_seg.pcd

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <fstream>

// ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
  //  not using as wee are not segmenting on live RTABMap feed
}
void calculate_mid_points(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  // Fixing ROS message into PCL Data type , for real time map input
  // pcl::fromROSMsg(*input, *cloud_XYZ);

  //  Current Approach is using Saved Point Cloud and then build pipeline

  pcl::io::loadPCDFile<pcl::PointXYZ>("src/uav-v01/uav_sim/pointcloud_maps/warehouse_legs.pcd", *cloud_XYZ);
  std::cout << "PointCloud file contains : " << cloud_XYZ->width * cloud_XYZ->height
            << " data points (" << pcl::getFieldsList(*cloud_XYZ) << ").\n";

  // ----

  //-- Passthrough filter for along Z Axis
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_XYZ);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0, 1.6);
  pass.filter(*cloud_XYZ_passthrough);
  //-- Passthrough filter for along X Axis
  pass.setInputCloud(cloud_XYZ_passthrough);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-1, 23);
  pass.filter(*cloud_XYZ_passthrough);
  //-- Create the voxel grid filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_XYZ_passthrough);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*cloud_XYZ_filtered);

  std::cerr << "PointCloud after voxel grid filtering: " << cloud_XYZ_filtered->width * cloud_XYZ_filtered->height << " data points (" << pcl::getFieldsList(*cloud_XYZ_filtered) << ").\n";

  //--  Segmentation using RanSac
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud_XYZ_filtered);
  seg.segment(*inliers, *coefficients);
  //  Extracting planners
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_XYZ_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_plane);
  // extracting the rest
  extract.setNegative(true);
  extract.filter(*cloud_f);
  *cloud_XYZ_filtered = *cloud_f;

  // For seperating legs by clustering
  pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  pcl::IndicesPtr indices(new std::vector<int>);
  std::vector<pcl::PointIndices> clusters;
  //   creating vectors to extracting mid points of racks
  std::vector<float> r1_mid_points;
  std::vector<float> r2_mid_points;
  std::vector<float> r3_mid_points;
  std::vector<float> all_midpoints;
  std::vector<std::pair<float, float>> cluster_init_values;
  std::vector<std::pair<float, float>> row_1_pair;
  std::vector<std::pair<float, float>> row_2_pair;
  std::vector<std::pair<float, float>> row_3_pair;

  // --- Calculating Normals and removing Nans -> not utilizing it
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud_f);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);
  pcl::removeNaNFromPointCloud(*cloud_f, *indices);

  // --- Applying RANSAC clustering algorithm
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(cloud_f);
  reg.setIndices(indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(60 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);
  reg.extract(clusters);

  // --- Getting information about our Clusters
  std::cout << "Total Clusters = " << clusters.size() << std::endl;

  for (int i = 0; i < clusters.size(); i++)
  {

    std::cout << "Cluster " << i << " Size =" << clusters[i].indices.size() << std::endl;
  }std::cout << "------------- " << std::endl;
  // -------

  //--- Extracting clusters using indices  from our cloud
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_pcl;
  std::vector<int> cluster_point_holder; // to hold starting points of all clusters
  for (int i = 0; i < clusters.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_cluster->width = clusters[i].indices.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    for (int j = 0; j < clusters[i].indices.size(); ++j)
    {
      cloud_cluster->push_back(cloud_f->at(clusters[i].indices[j]));
    }
    // Saving cluster first points into arrays
    cluster_init_values.push_back(std::make_pair(cloud_cluster->points[0].x, cloud_cluster->points[0].y));
  }
  //  Processing cluster values holders

  // Sorting pair vector along X
  sort(cluster_init_values.begin(), cluster_init_values.end());
  std::cout << "X Values\t"
            << "  Y Values" << std::endl;
  //  Printing Them Out
  for (int i = 0; i < cluster_init_values.size(); i++)
  {
    std::cout << cluster_init_values[i].first << "\t" << cluster_init_values[i].second << std::endl;
  }

  //  Segmenting all ROWS
  std::cout << "\nSegmenting all ROWS\n-----------------------------" << std::endl;
  for (int i = 0; i < 5; i++)
  {
    row_1_pair.push_back(cluster_init_values[i]);
    row_2_pair.push_back(cluster_init_values[i + 6]);
    row_3_pair.push_back(cluster_init_values[i + 11]);
  }

  std::cout << "\nFirst Row Segments\n-----------------------------" << std::endl;
  for (int i = 0; i < row_1_pair.size(); i++)
  {
    std::cout << row_1_pair[i].first << "\t" << row_1_pair[i].second << std::endl;
  }
  std::cout << "\nSecond Row Segments\n-----------------------------" << std::endl;
  for (int i = 0; i < row_2_pair.size(); i++)
  {
    std::cout << row_2_pair[i].first << "\t" << row_2_pair[i].second << std::endl;
  }

  std::cout << "\nThird Row Segments\n-----------------------------" << std::endl;
  for (int i = 0; i < row_3_pair.size(); i++)
  {
    std::cout << row_3_pair[i].first << "\t" << row_3_pair[i].second << std::endl;
  }

  // Sorted with Y
  std::cout << "\nSorting Along Y\n-----------------------------" << std::endl;
  std::sort(row_1_pair.begin(), row_1_pair.end(), [](auto &left, auto &right)
            { return left.second < right.second; });
  std::sort(row_2_pair.begin(), row_2_pair.end(), [](auto &left, auto &right)
            { return left.second < right.second; });
  std::sort(row_3_pair.begin(), row_3_pair.end(), [](auto &left, auto &right)
            { return left.second < right.second; });

  //  Printing Them Out
  std::cout << "X Values  "
            << "Y Values" << std::endl;
  for (int i = 0; i < row_1_pair.size(); i++)
  {
    std::cout << row_1_pair[i].first << "\t" << row_1_pair[i].second << std::endl;
  }

  // calculate Mid Points and save it in an Array

  for (int i = 0; i < row_1_pair.size() - 1; i++)
  {
    r1_mid_points.push_back((row_1_pair[i].second + row_1_pair[i + 1].second) / 2);
    // not calculating for others because it is going to produce same result
    // r2_mid_points.push_back((row_2_pair[i].second+row_2_pair[i+1].second) / 2 );
    // r3_mid_points.push_back((row_3_pair[i].second+row_3_pair[i+1].second) / 2 );
  }
  all_midpoints.insert(std::end(all_midpoints), std::begin(r1_mid_points), std::end(r1_mid_points));
  // not calculating for others because it is going to produce same result
  // all_midpoints.insert( std::end(all_midpoints),std::begin(r2_mid_points), std::end(r2_mid_points) );
  // all_midpoints.insert( std::end(all_midpoints),std::begin(r3_mid_points), std::end(r3_mid_points) );
  std::cout << "\nWare House Racks MidPoints\n-----------------------------" << std::endl;
  for (int i = 0; i < all_midpoints.size(); i++)
  {
    std::cout << all_midpoints[i] << std::endl;
  }

  //  Saving the vector into a text file
  fstream file;
  file.open("src/uav-v01/uav_sim/pointcloud_maps/processed/midpoints.txt", std::ios_base::out);
  for (auto content : all_midpoints)
  {
    file << content << endl;
  }

  file.close();

  std::cout << "Mid Poitns Written to file" << std::endl;
}


int main()
{ calculate_mid_points();
  return 0;

  // Create a ROS subscriber for the input point cloud when using RTABMAP live
  // ros::init(argc, argv, "rtabmap_pcl_segment");
  // ros::NodeHandle nh;
  // ros::Subscriber sub = nh.subscribe("/rtabmap/cloud_map", 1, cloud_cb);
  // ros::spin();
}