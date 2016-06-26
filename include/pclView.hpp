/**
 * Convert the disparity image into a pcl visualization
 */

#ifndef PCLVIEW_HPP
#define PCLVIEW_HPP

#include <General.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

class pclView {
public:
	pclView();
	~pclView();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	void loop_vizualizer();
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPointCloud(cv::Mat rgb_img, cv::Mat disparity_img, cv::Mat Q);
	pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointCloudNoColor(cv::Mat disparity_img, cv::Mat Q);
	void visualizeDisparityMap(cv::Mat rgb_img, cv::Mat disparity_img, cv::Mat Q);

private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerPtr;
};

#endif
