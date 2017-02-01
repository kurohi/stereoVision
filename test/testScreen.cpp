#include <screen.hpp>

int main(int argc, char **argv){
	Screen scr;
	char c = 0;
	while(c != 27){
		c = cv::waitKey(10);
	}
}
