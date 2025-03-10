# voxel_grid
Voxel Grid filtering of Lidar data

/*
Example of pcl usage where we try to use clustering to detect objects
in the scene.

Original code taken from:
http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction


Usage:
    first run: rosrun pcl_ros pcd_to_pointcloud <file.pcd> [ <interval> ]
    then run: rosrun my_pcl_tutorial cluster_extraction
    and look at the visualization through rviz with:
    rosrun rviz rviz

    Set /base_link as the Fixed Frame, check out the published PointCloud2 topics

*/
