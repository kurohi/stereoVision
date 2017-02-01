#ifndef PCLGETPLANES_HPP
#define PCLGETPLANES_HPP

#include "general.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>


class pclGetPlanes {
public:
	pclGetPlanes();
	pclGetPlanes(int indexes);
	~pclGetPlanes();
	bool computePaneByIndex(pcl::PointCloud<pcl::PointXYZ>::Ptr points, int index=-1);
	std::vector<double> getCoeficientsForIndex(int index);
	void setCoeficientsForIndex(std::vector<double> coefs, int index);
	pcl::PointIndices::Ptr getInlinersForIndex(int index);
	bool isOnPlane(pcl::PointXYZ point, int index);
	double distanceFromPlane(pcl::PointXYZ, int index);
	
	
	//in this case the indexes are decided between left, right, top, bottom
	//and back
	void cutImageForPlanes(cv::Mat src, cv::Mat &dst, int index);
	
	//Functions for dealing with loading and getting the points of a loaded planes
	bool addLoadedCoeficients(pcl::PointCloud<pcl::PointXYZ>::Ptr points, float a, float b, float c, float d, int index=-1);
	bool computeInliners(pcl::PointCloud<pcl::PointXYZ>::Ptr points, int index);
	
	//contants for class use
	static constexpr float CUTTING_PROPORTION=0.3;
	static constexpr int LEFT_CUT = 0;
	static constexpr int RIGHT_CUT = 1;
	static constexpr int TOP_CUT = 2;
	static constexpr int BOTTOM_CUT = 3;
	static constexpr int BACK_CUT = 4;
	static constexpr double THRESHOLD = 10.0;
private:
	int nIndexes;
	std::vector<pcl::ModelCoefficients::Ptr> planes;
	std::vector<pcl::PointIndices::Ptr> inliners;
};

#endif
