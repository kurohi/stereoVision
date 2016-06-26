#include <Screen.hpp>
#include <StereoDepth.hpp>

using namespace std;

int main(int argc, char **argv){
	Screen image1("Image1");
	Screen image2("Image2");
	Screen disp("Disparity");
	StereoDepth stereoDepth;
	char c = 0;
	Mat img1,img2;
	img1 = imread( argv[1] );
	img2 = imread( argv[2] );
	image1.putImage(img1);
	image2.putImage(img2);
	stereoDepth.setImage1(img1);
	stereoDepth.setImage2(img2);
	
	int minDisparity=-64;
	int numDisparities=192;
	int blockSize=5;
	int P1=600;
	int P2=2400;
	int disp12MaxDiff=10;
	int preFilterCap=4;
	int uniquenessRatio=1;
	int speckleWindowSize=150;
	int speckleRange=2;
	int mode=StereoSGBM::MODE_SGBM;
	
	while(c!=27){
		stereoDepth.doDepth();
		disp.putImage(stereoDepth.getDisparity());
		
		switch(c){
			case 'q':
				minDisparity++;
				//if(minDisparity>=0){
					stereoDepth.setMinDisparity(minDisparity);
				//}else{
				//	minDisparity = 0;
				//}
				cout<<"MinDisparity: "<<minDisparity<<endl;
				break;
			case 'a':
				minDisparity--;
				//if(minDisparity>=0){
					stereoDepth.setMinDisparity(minDisparity);
				//}else{
				//	minDisparity = 0;
				//}
				cout<<"MinDisparity: "<<minDisparity<<endl;
				break;
			case 'w':
				numDisparities+=16;
				if((numDisparities>0)&&((numDisparities%16)==0)){
					stereoDepth.setNumDisparities(numDisparities);
				}else{
					numDisparities=16;
				}
				cout<<"NumDisparities: "<<numDisparities<<endl;
				break;
			case 's':
				numDisparities-=16;
				if((numDisparities>0)&&((numDisparities%16)==0)){
					stereoDepth.setNumDisparities(numDisparities);
				}else{
					numDisparities=16;
				}
				cout<<"NumDisparities: "<<numDisparities<<endl;
				break;
			case 'e':
				blockSize+=2;
				if((blockSize>=1)&&((blockSize%2)==1)){
					stereoDepth.setBlockSize(blockSize);
				}else{
					blockSize = 1;
				}
				cout<<"BlockSize: "<<blockSize<<endl;
				break;
			case 'd':
				blockSize-=2;
				if((blockSize>=1)&&((blockSize%2)==1)){
					stereoDepth.setBlockSize(blockSize);
				}else{
					blockSize = 1;
				}
				cout<<"BlockSize: "<<blockSize<<endl;
				break;
			case 'r':
				P1++;
				if(P1<P2){
					stereoDepth.setP1(P1);
				}else{
					P1 = P2-1;
				}
				cout<<"P1: "<<P1<<endl;
				break;
			case 'f':
				P1--;
				if(P1<P2){
					stereoDepth.setP1(P1);
				}else{
					P1 = P2-1;
				}
				cout<<"P1: "<<P1<<endl;
				break;
			case 't':
				P2++;
				if(P2>P1){
					stereoDepth.setP2(P2);
				}else{
					P2 = P1+1;
				}
				cout<<"P2: "<<P2<<endl;
				break;
			case 'g':
				P2--;
				if(P2>P1){
					stereoDepth.setP2(P2);
				}else{
					P2 = P1+1;
				}
				cout<<"P2: "<<P2<<endl;
				break;
			case 'y':
				disp12MaxDiff++;
				stereoDepth.setDisp12MaxDiff(disp12MaxDiff);
				cout<<"disp12MaxDiff: "<<disp12MaxDiff<<endl;
				break;
			case 'h':
				disp12MaxDiff--;
				if(disp12MaxDiff<0){
					disp12MaxDiff = -1;
				}
				stereoDepth.setDisp12MaxDiff(disp12MaxDiff);
				cout<<"disp12MaxDiff: "<<disp12MaxDiff<<endl;
				break;
			case 'u':
				preFilterCap++;
				stereoDepth.setPreFilterCap(preFilterCap);
				cout<<"preFilterCap: "<<preFilterCap<<endl;
				break;
			case 'j':
				preFilterCap--;
				stereoDepth.setPreFilterCap(preFilterCap);
				cout<<"preFilterCap: "<<preFilterCap<<endl;
				break;
			case 'i':
				uniquenessRatio++;
				if(uniquenessRatio>0){
					stereoDepth.setUniquenessRatio(uniquenessRatio);
				}else{
					uniquenessRatio = 0;
				}
				cout<<"uniquenessRatio: "<<uniquenessRatio<<endl;
				break;
			case 'k':
				uniquenessRatio--;
				if(uniquenessRatio>0){
					stereoDepth.setUniquenessRatio(uniquenessRatio);
				}else{
					uniquenessRatio = 0;
				}
				cout<<"uniquenessRatio: "<<uniquenessRatio<<endl;
				break;
			case 'o':
				speckleWindowSize++;
				if(speckleWindowSize>0){
					stereoDepth.setSpeckleWindowSize(speckleWindowSize);
				}else{
					speckleWindowSize = 0;
				}
				cout<<"speckleWindowSize: "<<speckleWindowSize<<endl;
				break;
			case 'l':
				speckleWindowSize--;
				if(speckleWindowSize>0){
					stereoDepth.setSpeckleWindowSize(speckleWindowSize);
				}else{
					speckleWindowSize = 0;
				}
				cout<<"speckleWindowSize: "<<speckleWindowSize<<endl;
				break;
			case 'p':
				speckleRange++;
				if(speckleRange>0){
					stereoDepth.setSpeckleRange(speckleRange);
				}else{
					speckleRange = 0;
				}
				cout<<"speckleRange: "<<speckleRange<<endl;
				break;
			case ';':
				speckleRange--;
				if(speckleRange>0){
					stereoDepth.setSpeckleRange(speckleRange);
				}else{
					speckleRange = 0;
				}
				cout<<"speckleRange: "<<speckleRange<<endl;
				break;
			case '[':
				mode = StereoSGBM::MODE_HH;
				stereoDepth.setMode(mode);
				cout<<"Mode: Mode_HH"<<endl;
				break;
			case ']':
				mode = false;
				stereoDepth.setMode(mode);
				cout<<"Mode: false"<<endl;
				break;
			case 'z':
				cout<<"MinDisparity: "<<minDisparity<<endl;
				cout<<"BlockSize: "<<blockSize<<endl;
				cout<<"P1: "<<P1<<endl;
				cout<<"P2: "<<P2<<endl;
				cout<<"disp12MaxDiff: "<<disp12MaxDiff<<endl;
				cout<<"preFilterCap: "<<preFilterCap<<endl;
				cout<<"uniquenessRatio: "<<uniquenessRatio<<endl;
				cout<<"speckleWindowSize: "<<speckleWindowSize<<endl;
				cout<<"speckleRange: "<<speckleRange<<endl;
				if(mode == StereoSGBM::MODE_HH){
					cout<<"Mode: Mode_HH"<<endl;
				}else{
					cout<<"Mode: false"<<endl;
				}
				break;
		}
		
		c = waitKey(10);
	}
	Mat disp_img;
	disp_img = stereoDepth.getDisparity();
	equalizeHist(disp_img,disp_img);
	disp_img = abs(disp_img - 255);
	imwrite("result_img.jpg", disp_img);

}