
#include <ros/ros.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>


class SubscribeProcessPublish
{
public:
    SubscribeProcessPublish()
    {
        // assign subscriber
		this->subscriber = this->nh.subscribe<pcl::PCLPointCloud2>("/velodyne_points", 5, &SubscribeProcessPublish::processLidarMeasurementCallBack, this);
        
		// assign publisher
        this->publisher = this->nh.advertise<pcl::PCLPointCloud2> ("output", 1);

    }

	

	void 
    processLidarMeasurementCallBack(const pcl::PCLPointCloud2ConstPtr& cloud)
    {      
		//std::cout << "Received lidar measurement made at " << cloud->header.seq << std::endl;

		// define a new container for the data
		pcl::PCLPointCloud2::Ptr cloudVoxel (new pcl::PCLPointCloud2 ());
		
		// define a voxelgrid
		pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
		// set input to cloud
		voxelGrid.setInputCloud(cloud);
		// set the leaf size (x, y, z)
		voxelGrid.setLeafSize(0.2, 0.2, 0.2);
		// apply the filter to dereferenced cloudVoxel
		voxelGrid.filter(*cloudVoxel);

		//this->publisher.publish (*cloudVoxel);

		// cascade the floor removal filter and define a container for floorRemoved	
		pcl::PCLPointCloud2::Ptr floorRemoved (new pcl::PCLPointCloud2 ());
		
		// define a PassThrough
		pcl::PassThrough<pcl::PCLPointCloud2> pass;
		// set input to cloudVoxel
		pass.setInputCloud(cloudVoxel);
		// filter along z-axis

        pass.setFilterFieldName("x");
        pass.setFilterLimits(-10.0, 100.0);
        pass.filter (*floorRemoved);
        pass.setInputCloud (floorRemoved);

        pass.setFilterFieldName("y");
	//pass.setFilterLimits(-6.0, 10.0);
        pass.setFilterLimits(-10.0, 20.0);
        pass.filter (*floorRemoved);
        pass.setInputCloud (floorRemoved);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.2, 1.0);
        //pass.setFilterLimits(-0.7, 0.8);
        pass.filter(*floorRemoved);
        //pcl::getMinMax3D(*floorRemoved, min, max);
        //Eigen::Vector4f centroid;
        //pcl::compute3DCentroid(*floorRemoved, centroid);

        // Publish the data for visualisation
        this->publisher.publish (*floorRemoved);

		// define a point cloud of type PointXYZ
		pcl::PointCloud<pcl::PointXYZ> pclXYZ;

		// copy the contents of the floorRemoved to pclXYZ
		pcl::fromPCLPointCloud2(*floorRemoved, pclXYZ);
		//pcl::PointXYZ centroid;
				

    }

private:
    ros::NodeHandle nh;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
  	ros::ServiceClient client;
  	double tolerance;
};

int main(int argv, char** argc)
{
	// initialise the node	
	ros::init(argv, argc, "process_lidar");

	std::cout << "Process_lidar node initialised" << std::endl;

	// create instance of PublishSubscribe
	SubscribeProcessPublish process;

	// Handle ROS communication events
	ros::spin();

	return 0;
}

