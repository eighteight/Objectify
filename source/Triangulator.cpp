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

#include <pcl/io/vtk_lib_io.h>
#include <iostream>
#include "Triangulator.h"

using namespace boost;
using namespace pcl;
using namespace std;

    pcl::PolygonMesh triangles;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh mesh;

void Triangulator::triangulate(std::vector<std::vector<float> >& points, std::vector<std::vector<float> >& surfacePoints, vector<vector<int> >& triIndxs, int ksearchNeighbors, float gp3SearchRadius, int gp3MaxNeighbors, float gp3Mu){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    
    cloud->width  = points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize (points.size());
    
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        pcl::PointXYZ& pt = cloud->points[i];
        pt.z = points[i][0];
        pt.x = points[i][1];
        pt.y = points[i][2];
    }

    pcl::io::loadPolygonFile("plugins/Objectify/bun0.obj",mesh);
    
    //pcl::io::loadPolygonFile("/Users/eight/eliot2/eliot2_0000000.obj", mesh);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr  tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();

    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (ksearchNeighbors);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures
    
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals = make_shared<pcl::PointCloud<pcl::PointNormal> >();
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 = make_shared<pcl::search::KdTree<pcl::PointNormal> >();
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (gp3SearchRadius);

    // Set typical values for the parameters
    gp3.setMu (gp3Mu);

    gp3.setMaximumNearestNeighbors (gp3MaxNeighbors);

    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    for (int t = 0; t < triangles.polygons.size(); t++){
        cout<<triangles.polygons[t].vertices.size()<<endl;
        vector<int> ind(3);
        ind[0] = triangles.polygons[t].vertices[0];
        ind[1] = triangles.polygons[t].vertices[1];
        ind[2] = triangles.polygons[t].vertices[2];
        triIndxs.push_back(ind);
    }
    
    for (int j = 0; j < cloud->points.size(); j++){
        vector<float>p(3);
        p[0] = cloud->points[j].x;
        p[1] = cloud->points[j].y;
        p[2] = cloud->points[j].z;
        surfacePoints.push_back(p);
    }
    
    return;
    
    std::cout<<triangles<<std::endl;
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    std::cout<<&parts<<std::endl;    
}
