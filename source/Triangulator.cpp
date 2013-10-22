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
#include <pcl/io/vtk_lib_io.h>
#include <vtkSmartPointer.h>
#include <pcl/io/obj_io.h>
#include <pcl/ros/conversions.h>
#include <iostream>
#include "Triangulator.h"
void Triangulator::triangulate(){
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFile("cube.obj",mesh);
    
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    
    //    pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
    //    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
    //* the data should be available in cloud
    
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures
    
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
    
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);
    
    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    
    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
    
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    
    //pcl::PolygonMesh polygon_mesh;
    //pcl::fromPCLPointCloud2(*cloud_, triangles.cloud);
    //triangles.polygons = polygons_;
    
    ////////
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    //cloud1 = triangles.cloud;
    // Fill in the cloud data
    cloud1.width    = 1;
    cloud1.height   = 1;
    cloud1.is_dense = false;
    cloud1.points.resize(cloud1.width * cloud1.height);
    
    for (size_t i = 0; i < cloud1.points.size (); ++i)
    {
        cloud1.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
        cloud1.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
        cloud1.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
    }
    
    pcl::io::savePCDFile("out.pcd", cloud1, false);
    ///////
    
    
    
    //pcl::PCLPointCloud2<pcl::PointXYZ> &cloud_;
    //pcl::toROSMsg(triangles.cloud, cloud_);
    
    //pcl::io::savePCDFile("out.pcd", *cloud_, true);
    pcl::io::saveOBJFile("out.obj", triangles, true);
}