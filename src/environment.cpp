/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors


#include "render/render.h"
#include "quiz/cluster/kdtree.h"
#include "processPointClouds.h"
#include "sensors/lidar.h"
#include <unordered_set>

// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	

	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size()<3)
			inliers.insert(rand()%(cloud->points.size()));
		
		float x1,y1,z1,x2,y2,z2,x3,y3,z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float a = ((y2-y1)*(z3-z1) - (z2-z1)*(y3-y1));
		float b = ((z2-z1)*(x3-x1) - (x2-x1)*(z3-z1));
		float c = ((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1));
		float d = -(a*x1 + b*y1 + c*z1);

		for(int i=0;i<cloud->points.size(); i++)
		{
			if(inliers.count(i)>0)
				continue;
			pcl::PointXYZI point = cloud->points[i];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			float D = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);

			if(D<distanceTol)
				
				inliers.insert(i);

		}
			
			if(inliers.size() > inliersResult.size()){
				inliersResult = inliers;
			}
		
		
		

	}
	
	
	return inliersResult;

}




void Proximity(int Ind,const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{

	processed[Ind] = true;
	cluster.push_back(Ind);

	std::vector<int> nearby = tree->search(points[Ind], distanceTol);

	for(int id : nearby)
	{
		
		if(!processed[id])
			Proximity(id, points, cluster, processed, tree, distanceTol);
	}
	
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	int j = 0;
	while(j<points.size()){
		
		if(processed[j]){
			j++;
			continue;
		}

		std::vector<int> cluster;
		Proximity(j, points, cluster,processed, tree, distanceTol);
		clusters.push_back(cluster);
		j++;
	}

	return clusters;

}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering_from_stratch(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTol, int minsize, int maxsize)
{
    
    
    KdTree* tree = new KdTree; 
    std::vector<std::vector<float>> data;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> final_clusters;

    for(int i = 0; i < cloud->points.size(); i++)
    {
        //pcl::PointT point = cloud->points[i];
      	std::vector<float> VectorPoint = {cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
        data.push_back(VectorPoint);
      	tree->insert(VectorPoint,i);
      
    }
    
    std::vector<std::vector<int>> clusters = euclideanCluster(data, tree, 0.70);
    
    
    
    
    for(std::vector<int> cluster:clusters)
    {   
        if(cluster.size() < minsize || cluster.size()> maxsize)
            {continue;}

        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        
        for(int indice:cluster)
        {
            
            clusterCloud->points.push_back(cloud->points[indice]);
        }

        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
    
        final_clusters.push_back(clusterCloud);

    }  
return final_clusters;

}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool rendercluster = true;
    bool renderBBox = false; 
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* mylidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = mylidar->scan();
    //renderRays(viewer,mylidar->position,input);
    renderPointCloud(viewer,input,"point_cloud");
    // TODO:: Create point processor

    ProcessPointClouds<pcl::PointXYZ> pointprocessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr , pcl::PointCloud <pcl::PointXYZ>::Ptr> segmentcloud = pointprocessor.SegmentPlane(input,100,0.2);
    //renderPointCloud(viewer,segmentcloud.first,"obstcloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentcloud.second,"roadcloud",Color(0,1,0));
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudCluster = pointprocessor.Clustering(segmentcloud.first,1.3,3,80 );
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  int clusterId = 0;
  for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudCluster)
  {
    if(rendercluster)
    {
    std::cout<< "cluster size";
    pointprocessor.numPoints(cluster);
    renderPointCloud(viewer, cluster,"Obstcloud"+std::to_string(clusterId),colors[clusterId]);
    }

    if(renderBBox){

        Box box = pointprocessor.BoundingBox(cluster);
        renderBox(viewer, box,clusterId);
    }
    ++clusterId;
  } 
  

  
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointprocessor, pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    //ProcessPointClouds<pcl::PointXYZI> pointprocessor;

    

    //pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = pointprocessor.loadPcd("/home/workspace/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, input_cloud,"inputcloud");

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtercloud = pointprocessor.FilterCloud(input_cloud, 0.25f, Eigen::Vector4f(-20.0,-5.0,-4.0,1.0), Eigen::Vector4f(20.0,7.0,5.0,1.0));
    
    /* commented lines of inbuilt PCL functions
    ----------------------------------------------------------------------------------------------------------------
    
    //renderPointCloud(viewer, filtercloud, "downsampled_cloud");
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr , pcl::PointCloud <pcl::PointXYZI>::Ptr> segmentcloud = pointprocessor.SegmentPlane(filtercloud,100,0.25);
    
    -----------------------------------------------------------------------------------------------------------------*/
    
    /* Using RANSAC from stratch function */
    
    std::unordered_set<int> inliers = Ransac(filtercloud, 100, 0.25);

	pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<std::vector<float>> data;
	for(int index = 0; index < filtercloud->points.size(); index++)
	{
		pcl::PointXYZI point = filtercloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
              
	}
  

    
    
    //renderPointCloud(viewer,cloudOutliers,"obstcloud",Color(1,0,0));
    renderPointCloud(viewer,cloudInliers,"roadcloud",Color(0,1,0));

    /* commented inbuilt PCL Clustering functions

    -----------------------------------------------------------------------------------------------------------------------

    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudCluster = pointprocessor.Clustering(cloudOutliers,0.70,15,450);
    //std::vector<pcl::PointXYZI> data = cloudOutliers->points;

    -----------------------------------------------------------------------------------------------------------------------*/

    // Clustering from stratch function
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = Clustering_from_stratch(cloudOutliers,0.70,15,450);
    int clusterId = 0;
    for(auto cluster:clusters)
    {
        /* code */

        std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
        renderPointCloud(viewer, cluster,"Obstcloud"+std::to_string(clusterId),colors[clusterId%3]);
        Box box = pointprocessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI> pointprocessorI;
    std::vector<boost::filesystem::path> stream = pointprocessorI.streamPcd("/home/workspace/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud; 
    //cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        inputcloud = pointprocessorI.loadPcd((*streamIterator).string());
        cityBlock(viewer,pointprocessorI,inputcloud);
        streamIterator++;

        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}