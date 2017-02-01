/**
 *  implementation of the pclView class
 */

#include "pcl_view.hpp"

pclView::pclView(){
}

pclView::~pclView(){

}

boost::shared_ptr<pcl::visualization::PCLVisualizer> pclView::createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
	viewer->addCoordinateSystem ( 1.0 );
	viewer->initCameraParameters ();
	viewerPtr = viewer;
	return viewer;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclView::convertToPointCloud(cv::Mat img_rgb, cv::Mat img_disparity, cv::Mat Q){
	//Get the interesting parameters from Q
	double Q03, Q13, Q23, Q32, Q33;
	Q03 = Q.at<double>(0,3);
	Q13 = Q.at<double>(1,3);
	Q23 = Q.at<double>(2,3);
	Q32 = Q.at<double>(3,2);
	Q33 = Q.at<double>(3,3);
	
	//Both images must be same size
	if (img_rgb.size() != img_disparity.size())
	{
		std::cerr << "ERROR: rgb-image and disparity-image have different sizes " << std::endl;
		exit(1);
	}
		
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	double px, py, pz;
	uchar pr, pg, pb;
	
	for (int i = 0; i < img_rgb.rows; i++)
	{
		for (int j = 0; j < img_rgb.cols; j++)
		{
			//Get 3D coordinates
			uchar d = img_disparity.at<uchar>(i,j);
			if ( d == 0 ) continue; //Discard bad pixels
			double pw = -1.0 * static_cast<double>(d) * Q32 + Q33; 
			px = static_cast<double>(j) + Q03;
			py = static_cast<double>(i) + Q13;
			pz = Q23;

			px = px/pw;
			py = py/pw;
			pz = pz/pw;

			//Get RGB info
			pb = img_rgb.at<cv::Vec3b>(i,j).val[0];
			pg = img_rgb.at<cv::Vec3b>(i,j).val[1];
			pr = img_rgb.at<cv::Vec3b>(i,j).val[2];

			//Insert info into point cloud structure
			pcl::PointXYZRGB point;
			point.x = px;
			point.y = py;
			point.z = pz;
			uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
							static_cast<uint32_t>(pg) << 8 | 
							static_cast<uint32_t>(pb));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back (point);
		}

	}
	point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;
	
	return point_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pclView::convertToPointCloudNoColor(cv::Mat img_disparity, cv::Mat Q){
	//Get the interesting parameters from Q
	double Q03, Q13, Q23, Q32, Q33;
	Q03 = Q.at<double>(0,3);
	Q13 = Q.at<double>(1,3);
	Q23 = Q.at<double>(2,3);
	Q32 = Q.at<double>(3,2);
	Q33 = Q.at<double>(3,3);

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

	double px, py, pz;
	uchar pr, pg, pb;
	
	for (int i = 0; i < img_disparity.rows; i++)
	{
		for (int j = 0; j < img_disparity.cols; j++)
		{
			//Get 3D coordinates
			uchar d = img_disparity.at<uchar>(i,j);
			if ( d == 0 ) continue; //Discard bad pixels
			double pw = -1.0 * static_cast<double>(d) * Q32 + Q33; 
			px = static_cast<double>(j) + Q03;
			py = static_cast<double>(i) + Q13;
			pz = Q23;

			px = px/pw;
			py = py/pw;
			pz = pz/pw;

			//Insert info into point cloud structure
			pcl::PointXYZ point;
			point.x = px;
			point.y = py;
			point.z = pz;

			point_cloud_ptr->points.push_back (point);
		}

	}
	point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;
	
	return point_cloud_ptr;
}

void pclView::visualizeDisparityMap(cv::Mat img_rgb, cv::Mat img_disparity, cv::Mat Q){
	try{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (convertToPointCloud(img_rgb, img_disparity, Q));
		createVisualizer( point_cloud_ptr );
  
		//Main loop
		while ( !viewerPtr->wasStopped())
		{
			viewerPtr->spinOnce(100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
	}catch(cv::Exception& e){
		std::cout << e.what() << std::endl;
	}
}

void pclView::loop_vizualizer(){
	//Main loop
	while ( !viewerPtr->wasStopped())
	{
		viewerPtr->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}
