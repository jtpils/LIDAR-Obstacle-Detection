/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	int cloud_size = cloud->points.size();
	// For max iterations 
	for(int index=0; index<maxIterations; index++) {
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers_temp;
		while(inliers_temp.size() < 3) {
			inliers_temp.insert(rand()%cloud_size);
		}
		auto itr = inliers_temp.begin();	
		pcl::PointXYZ one = cloud->points[*itr];
		itr++;
		pcl::PointXYZ two = cloud->points[*itr];
		itr++;
		pcl::PointXYZ three = cloud->points[*itr];
		// fit line for the model
		std::vector<float> vec_one {two.x-one.x, two.y-one.y, two.z-one.z};
		std::vector<float> vec_two {three.x-one.x, three.y-one.y, three.z-one.z};

		// find the normal vector of vectors vec_one and vec_two
		float coefficient_A = vec_one[1]*vec_two[2] - vec_one[2]*vec_two[1];
		float coefficient_B = vec_one[2]*vec_two[0] - vec_one[0]*vec_two[2];
		float coefficient_C = vec_one[0]*vec_two[1] - vec_one[1]*vec_two[0];
		float coefficient_D = -(coefficient_A*one.x + coefficient_B*one.y + coefficient_C*one.z);

		// float coefficient_A = one.y - two.y;
		// float coefficient_B = two.x - one.x;
		// float coefficient_C = one.x*two.y - two.x*one.y;
		for (int ind = 0; ind<cloud_size; ind++) {
			// Measure distance between every point and fitted line
			if (inliers_temp.count(ind) > 0) 
				continue;
			pcl::PointXYZ points = cloud->points[ind];
			float distance = fabs((coefficient_A*points.x + coefficient_B*points.y + coefficient_C*points.z + coefficient_D)/
									(sqrt(coefficient_A*coefficient_A + coefficient_B*coefficient_B + coefficient_C*coefficient_C)));
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol) {
				inliers_temp.insert(ind);
			}
		}
		if (inliers_temp.size() > inliersResult.size()) {
			inliersResult = inliers_temp;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D(); 
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 500, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
