#include <pclGetPlanes.hpp>

pclGetPlanes::pclGetPlanes(){
	nIndexes = 0;
}
pclGetPlanes::pclGetPlanes(int indexes){
	nIndexes = indexes;
	for(int i=0; i<indexes; i++){
		pcl::ModelCoefficients::Ptr modelPointer(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inlinerPointer (new pcl::PointIndices);
		planes.push_back(modelPointer);
		inliners.push_back(inlinerPointer);
	}
}

pclGetPlanes::~pclGetPlanes(){
	planes.clear();
	inliners.clear();
}

bool pclGetPlanes::computePaneByIndex(pcl::PointCloud<pcl::PointXYZ>::Ptr points, int index){
	if((index==-1)||(index>=planes.size())){
		pcl::ModelCoefficients::Ptr modelPointer(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inlinerPointer (new pcl::PointIndices);
		planes.push_back(modelPointer);
		inliners.push_back(inlinerPointer);
	}
	if(index==-1){
		index = planes.size()-1;
	}
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//Optional
	seg.setOptimizeCoefficients (true);
	//Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	//seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (THRESHOLD);
	
	seg.setInputCloud (points);
	seg.segment (*(inliners[index]), *(planes[index]));
	if (inliners[index]->indices.size() == 0) {
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return false;
	}
	return true;
}


std::vector<double> pclGetPlanes::getCoeficientsForIndex(int index){
	if((index<0)||(index>=planes.size())){
		PCL_ERROR ("Invalid index provided");
		throw;
	}
	std::vector<double> coeficients;
	coeficients.push_back(planes[index]->values[0]);
	coeficients.push_back(planes[index]->values[1]);
	coeficients.push_back(planes[index]->values[2]);
	coeficients.push_back(planes[index]->values[3]);
	return coeficients;
	
}

void pclGetPlanes::setCoeficientsForIndex(std::vector<double> coefs, int index){
	if(coefs.size() < 4){
		throw(std::runtime_error("invalid plane coeficients"));
	}
	if((index==-1)||(index>=planes.size())){
		pcl::ModelCoefficients::Ptr modelPointer(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inlinerPointer (new pcl::PointIndices);
		planes.push_back(modelPointer);
		inliners.push_back(inlinerPointer);
	}
	if(index==-1){
		index = planes.size()-1;
	}
	for(int i=0; i<4; i++){
		if(i>=planes[index]->values.size()){
			planes[index]->values.push_back(coefs[i]);
		}else{
			planes[index]->values[i] = coefs[i];
		}
	}
}


pcl::PointIndices::Ptr pclGetPlanes::getInlinersForIndex(int index){
	if((index<0)||(index>=inliners.size())){
		PCL_ERROR ("Invalid index provided");
		throw;
	}
	return inliners[index];
}

void pclGetPlanes::cutImageForPlanes(cv::Mat src, cv::Mat &dst, int index){
	dst = cv::Mat::zeros(src.rows,src.cols, CV_8UC1);
	switch(index){
		case pclGetPlanes::LEFT_CUT:
			for(int i=0; i<src.rows; i++)
				for(int j=0; j<src.cols*CUTTING_PROPORTION; j++){
					dst.at<uchar>(i,j) = src.at<uchar>(i,j);
				}
			break;
		case pclGetPlanes::RIGHT_CUT:
			for(int i=0; i<src.rows; i++)
				for(int j=src.cols - src.cols*CUTTING_PROPORTION; j<src.cols; j++){
					dst.at<uchar>(i,j) = src.at<uchar>(i,j);
				}
			break;
		case pclGetPlanes::TOP_CUT:
			for(int i=0; i<src.rows*CUTTING_PROPORTION; i++)
				for(int j=0; j<src.cols; j++){
					dst.at<uchar>(i,j) = src.at<uchar>(i,j);
				}
			break;
		case pclGetPlanes::BOTTOM_CUT:
			for(int i=src.rows - src.rows*CUTTING_PROPORTION; i<src.rows; i++)
				for(int j=0; j<src.cols; j++){
					dst.at<uchar>(i,j) = src.at<uchar>(i,j);
				}
			break;
		case pclGetPlanes::BACK_CUT:
			for(int i=src.rows*CUTTING_PROPORTION; i<src.rows - src.rows*CUTTING_PROPORTION; i++)
				for(int j=src.cols*CUTTING_PROPORTION; j<src.cols - src.cols*CUTTING_PROPORTION; j++){
					dst.at<uchar>(i,j) = src.at<uchar>(i,j);
				}
			break;
	}
}

bool pclGetPlanes::addLoadedCoeficients(pcl::PointCloud<pcl::PointXYZ>::Ptr points, float a, float b, float c, float d, int index){
	if((index==-1)||(index>=planes.size())){
		pcl::ModelCoefficients::Ptr modelPointer(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inlinerPointer (new pcl::PointIndices);
		planes.push_back(modelPointer);
		inliners.push_back(inlinerPointer);
	}else if(index>planes.size()){
		PCL_ERROR ("Invalid index provided. Too big.");
		return false;
	}else{
		planes[index]->values.clear();
	}
	planes[index]->values.push_back(a);
	planes[index]->values.push_back(b);
	planes[index]->values.push_back(c);
	planes[index]->values.push_back(d);
	
	return computeInliners(points, index);
}

bool pclGetPlanes::computeInliners(pcl::PointCloud<pcl::PointXYZ>::Ptr points, int index){
	if((index==-1)||(index>=planes.size())){
		PCL_ERROR ("Invalid index provided.");
		return false;
	}
	// recalculating the inliners
	
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr dit (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (points)); 
	
	Eigen::Vector4f coefficients = Eigen::Vector4f(planes[index]->values[0], planes[index]->values[1], planes[index]->values[2], planes[index]->values[3]); 
	std::vector<int> recalcInliner; 
	dit -> selectWithinDistance (coefficients, THRESHOLD, recalcInliner); 
	
	inliners[index]->indices.clear();
	inliners[index]->indices = recalcInliner;
	return true;
}

bool pclGetPlanes::isOnPlane(pcl::PointXYZ point, int index){
	return pcl::pointToPlaneDistance(point, planes[index]->values[0],planes[index]->values[1], planes[index]->values[2], planes[index]->values[3]) < THRESHOLD;
}

double pclGetPlanes::distanceFromPlane(pcl::PointXYZ point, int index){
	return pcl::pointToPlaneDistance(point, planes[index]->values[0],planes[index]->values[1], planes[index]->values[2], planes[index]->values[3]);
}
