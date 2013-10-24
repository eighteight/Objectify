//
//  Triangulator.cpp
//  objectify
//
//  Created by Gusev, Vladimir on 10/22/13.
//
//


#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <boost/make_shared.hpp>
#include <iostream>
#include "Triangulator.h"
int user_data;

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer"));

//pcl::visualization::PCLVisualizer viewer ("Test: NURBS surface fitting");


    
    //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

void Triangulator::triangulate(std::vector<std::vector<float> >& points){

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    
//    cloud->width  = points.size();
//    cloud->height = 1;
//    cloud->is_dense = false;
//    cloud->points.resize (points.size());
//    
//    for (size_t i = 0; i < cloud->points.size (); ++i)
//    {
//        pcl::PointXYZ& pt = cloud->points[i];
//        pt.z = points[i][0];
//        pt.x = points[i][1];
//        pt.y = points[i][2];
//    }
    
    // Normal estimation*
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr  bTree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud (cloud);
//    n.setInputCloud (cloud);
//    n.setSearchMethod (tree);
//    n.setKSearch (20);
//    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures
    
    // Concatenate the XYZ and normal fields*
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    
    // Create search tree*
//    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//    tree2->setInputCloud (cloud_with_normals);
    
    // Initialize objects
//      pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//    pcl::PolygonMesh triangles;
//    
//    // Set the maximum distance between connected points (maximum edge length)
//    gp3.setSearchRadius (0.025);
//    
//    // Set typical values for the parameters
//    gp3.setMu (2.5);
//    gp3.setMaximumNearestNeighbors (100);
//    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//    gp3.setMinimumAngle(M_PI/18); // 10 degrees
//    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//    gp3.setNormalConsistency(false);
//    
//    // Get result
//    gp3.setInputCloud (cloud_with_normals);
//    gp3.setSearchMethod (tree2);
//    gp3.reconstruct (triangles);
//    
//    std::cout<<triangles<<std::endl;
//    // Additional vertex information
//    std::vector<int> parts = gp3.getPartIDs();
//    std::vector<int> states = gp3.getPointStates();
//    std::cout<<&parts<<std::endl;
    
    //pcl::visualization::PCLVisualizer& viewer;
    
//    viewer.setSize(300, 300);
//    viewer.addPolygonMesh(triangles, "triangles");
//      viewer.spin ();
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb (colored_cloud);
    
    
}